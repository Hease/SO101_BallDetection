# -*- coding: utf-8 -*-
"""robot.py — 시뮬과 실물을 같은 코드로 움직이는 제어 계층.

soarm_lab 을 그대로 쓰지 못하고 이 파일이 필요한 이유가 세 가지 있다.

1. `Arm.run(states)` 은 내부에서 `_backend(False)` 를 부른다 — **시뮬 전용이다.**
   examples/pick_place.py 의 순서표 패턴은 실물 팔을 한 밀리미터도 못 움직인다.
2. `SimBackend.move()` 는 물리를 돌리며 `time.sleep` 한다 — **블로킹이다.**
   GUI 스레드에서 부르면 창이 얼어붙는다. 그래서 논블로킹인 `LiveSim` 을 쓴다.
3. `RealBackend.move()` 는 **그리퍼(id 6)를 아예 건드리지 않는다.**
   주석만 있고 구현이 없어, 실물로는 공을 잡을 수가 없다.

여기서 하는 일은 그래서: 논블로킹 백엔드 위에 '도착 대기'를 얹고, 그리퍼
percent 매핑을 채우고, 둘을 한 API 로 덮는 것. soarm_lab 자체는 수정하지 않는다.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass

import numpy as np

from . import config as cfg

ARM_IDS = (1, 2, 3, 4, 5)
GRIP_ID = 6
ADDR_MOVING = 66          # driver_sdk 에 상수만 있고 접근자가 없어 여기서 직접 읽는다


class OutOfReach(Exception):
    """IK 가 그 좌표를 못 푸는 경우. 공 하나를 건너뛰되 사이클은 계속한다."""


@dataclass
class GripMap:
    """그리퍼 fraction(0=닫힘, 1=열림) ↔ 시뮬 각도 / 서보 raw (표시·리드백용).

    실물 '작동'은 real.py.grip(frac) 이 맡는다(정본). 여기 매핑은 real.py 에 없는
    나머지 — 시뮬 각도(ctrl[5]), 3D 트윈 표시각, 서보 raw→frac 리드백 — 에만 쓴다.
    실물 그리퍼는 서보 한 바퀴가 집게 ~110° 로 줄어드는 링키지라, driver_sdk.
    JOINT_LIMITS[6] 이 그 보정값을 갖고 있다(min/max=실측 raw 양끝, flip=역방향 팔).
    """
    raw_min: int
    raw_max: int
    deg_min: float
    deg_max: float
    flip: bool = False

    @classmethod
    def from_driver(cls) -> "GripMap":
        from soarm_lab.driver_sdk import JOINT_LIMITS
        lim = JOINT_LIMITS[GRIP_ID]
        return cls(
            raw_min=int(lim["min"]), raw_max=int(lim["max"]),
            deg_min=math.degrees(float(lim.get("rad_min", -0.175))),
            deg_max=math.degrees(float(lim.get("rad_max", 1.745))),
            flip=bool(lim.get("flip", False)),
        )

    def to_deg(self, frac: float) -> float:
        t = max(0.0, min(1.0, frac))
        return self.deg_min + t * (self.deg_max - self.deg_min)

    def raw_to_frac(self, raw: int) -> float:
        span = self.raw_max - self.raw_min
        if span == 0:
            return 0.0
        t = (raw - self.raw_min) / span
        if self.flip:
            t = 1.0 - t
        return max(0.0, min(1.0, t))


class RobotController:
    """좌표를 주면 팔이 가고, 다 갈 때까지 기다려 주는 계층.

    real=False 면 헤드리스 MuJoCo(LiveSim), True 면 실물 서보. 위쪽 코드
    (pipeline·GUI)는 어느 쪽인지 몰라도 된다.

    **시리얼은 이 객체를 소유한 스레드에서만 만져야 한다.** driver_sdk 는
    sync write 없이 관절마다 개별 write 를 하므로, 두 스레드가 동시에 쓰면
    패킷이 섞인다.
    """

    def __init__(self, real: bool = False, slow: bool = False,
                 conf: cfg.RobotConfig | None = None):
        self.conf = conf or cfg.ROBOT
        self.real = real
        self.slow = slow
        self._estopped = False
        self._grip_frac = self.conf.grip_open
        self._last_deg = list(self.conf.home_pose_deg)

        # 이동 도중 "지금 당장 멈춰야 하나?"를 물어보는 훅. 파이프라인이 침입
        # 감지와 딜레이 워치독을 여기 연결한다. 이게 없으면 팔이 한 번 움직이기
        # 시작한 뒤로는 그 동작이 끝날 때까지 손이 들어와도 알아채지 못한다.
        self.should_abort = None

        # 실측한 작업영역. 아직 안 쟀으면 None 이고, 그 경우 이 층의 검사는
        # 건너뛴다(IK 잔차와 관절 클램프는 그대로 남는다).
        from .workspace import Workspace
        self.workspace = Workspace.load()

        if not real:
            os.environ.setdefault("SOARM_HEADLESS", "1")   # 트윈은 GUI가 따로 그린다

        from soarm_lab import arm as _arm
        self.arm = _arm
        if not real:
            self.arm.live()          # 논블로킹 백엔드 (SimBackend 는 블로킹이라 못 쓴다)

        self.backend = self.arm._backend(real)
        self.drv = getattr(self.backend, "drv", None)
        self.grip_map = GripMap.from_driver()
        # LiveSim 은 물리를 백그라운드 스레드에서 돈다 — data 를 직접 만질 땐 이 락을 쥔다
        self._sim_lock = getattr(self.backend, "_lock", None)

        if real:
            self._init_servos()

    def _sim_data(self):
        """LiveSim 의 MjData 를 락과 함께 쓰기 위한 컨텍스트."""
        import contextlib
        lock = self._sim_lock
        return lock if lock is not None else contextlib.nullcontext()

    # ── 초기화 ────────────────────────────────────────────────────────────
    def _init_servos(self) -> None:
        """토크를 켜고 속도·가감속을 건다.

        토크를 다시 켜는 이유: 캘리브레이션(티칭)이 토크를 꺼둔 채 끝나므로,
        그대로 두면 명령을 보내도 팔이 축 늘어져 있다.
        """
        if not self.drv.ping(1):
            raise SystemExit(
                f"서보가 응답하지 않습니다(id1, {self.conf.port}).\n"
                "로봇 전원·USB 케이블을 확인하고, 포트를 쓰는 다른 프로그램을 종료하세요.")
        speed = self.conf.slow_speed if self.slow else self.conf.speed
        accel = self.conf.slow_accel if self.slow else self.conf.accel
        for sid in ARM_IDS + (GRIP_ID,):
            self.drv.set_torque(sid, True)
            self.drv.set_acceleration(sid, accel)
            self.drv.set_speed(sid, speed)

    # ── 상태 조회 ─────────────────────────────────────────────────────────
    def joint_angles_deg(self) -> list[float]:
        """3D 트윈에 보낼 관절각 6개(팔 5 + 그리퍼).

        실물에서 매 렌더마다 서보를 읽으면 시리얼이 명령과 경합하므로,
        마지막으로 명령했거나 읽어둔 값을 돌려준다.
        """
        return list(self._last_deg) + [self.grip_map.to_deg(self._grip_frac)]

    def read_arm_deg(self) -> list[float] | None:
        """실물 서보에서 실제 각도를 읽는다(느리다 — 도착 판정에만 쓴다)."""
        if not self.real:
            with self._sim_data():
                return [float(np.degrees(a)) for a in self.backend.data.qpos[:5]]
        pos = self.drv.get_all_positions()
        from soarm_lab.driver_sdk import STS3215Driver
        out = []
        for sid in ARM_IDS:
            raw = pos.get(sid)
            if raw is None:
                return None
            out.append(STS3215Driver.position_to_degrees(raw) or 0.0)
        self._last_deg = out
        return out

    # ── 그리퍼 ─────────────────────────────────────────────────────────────
    def set_grip(self, frac: float, settle: float = 0.4) -> None:
        """그리퍼를 frac(0=닫힘, 1=열림)으로. 너무 빨리 닫으면 공을 튕겨낸다.

        실물 작동은 real.py 의 grip(frac) 이 정본이다(팀 합의). 여기서 raw 매핑을
        다시 하지 않고 backend.grip 에 넘긴다 — 값은 양쪽 다 frac(0~1)이라 변환 없음.
        시뮬 백엔드(LiveSim)에는 grip 이 없으므로 ctrl[5] 경로는 그대로 둔다.
        표시·리드백용 frac↔deg/raw 변환은 GripMap 에 남는다(real.py 엔 그게 없다).
        """
        frac = max(0.0, min(1.0, frac))
        self._grip_frac = frac
        if self.real:
            self.backend.grip(frac)                       # 정본: real.py.RealBackend.grip
        else:
            with self._sim_data():
                self.backend.data.ctrl[5] = math.radians(self.grip_map.to_deg(frac))
        if settle:
            self._sleep(settle)

    def _sleep(self, secs: float) -> None:
        """중단 요청을 놓치지 않는 sleep. 통째로 자면 그 사이 손이 들어와도 못 본다."""
        deadline = time.monotonic() + secs
        while time.monotonic() < deadline:
            if self._estopped:
                return
            if self.should_abort is not None and self.should_abort():
                self.estop()
                return
            time.sleep(0.02)

    def grip_frac_actual(self) -> float | None:
        """실제 그리퍼 개도(0~1). 파지 성공 판정에 쓴다."""
        if not self.real:
            with self._sim_data():
                deg = float(np.degrees(self.backend.data.qpos[5]))
            span = self.grip_map.deg_max - self.grip_map.deg_min
            if span == 0:
                return None
            return max(0.0, min(1.0, (deg - self.grip_map.deg_min) / span))
        raw = self.drv.get_position(GRIP_ID)
        return None if raw is None else self.grip_map.raw_to_frac(raw)

    def holding_object(self) -> bool:
        """공을 실제로 물고 있는가.

        닫으라고 명령했는데 집게가 '완전히' 닫혀 버렸다면 사이에 아무것도 없는
        것이다. 공이 물려 있으면 그 두께만큼 덜 닫힌다.
        """
        actual = self.grip_frac_actual()
        if actual is None:
            return True            # 못 읽으면 성공으로 가정 — 비전 재확인에 맡긴다
        return actual > self.conf.grip_empty_frac

    # ── 이동 ──────────────────────────────────────────────────────────────
    def check_workspace(self, xyz):
        """실측 작업영역으로 좌표를 검사한다. 아직 안 쟀으면 None.

        측정 전에 그럴듯한 기본 영역을 지어내지 않는다 — 없는 한계를 있는 척하면
        "한계 검사가 돌고 있다"고 착각하게 된다. GUI 가 미측정을 눈에 띄게 알린다.
        """
        if self.workspace is None:
            return None
        return self.workspace.check(float(xyz[0]), float(xyz[1]), float(xyz[2]))

    def move_to(self, xyz, down: bool = False, wait: bool = True) -> None:
        """손끝을 좌표로. 도착할 때까지 기다린다(wait=False 면 명령만).

        **3중 방어를 여기서 순서대로 통과시킨다.** 각 층이 잡는 것이 다르다.

          ① 실측 작업영역 — IK 를 부르기 *전에* 거절한다. 빠르고 예측 가능하며
             무엇보다 화면에 그릴 수 있다. IK 는 반경 0.44m 까지 잔차 0mm 로
             풀어내므로 이 층이 없으면 사실상 한계가 없다.
          ② IK 잔차(`OutOfReach`) — 영역 안이지만 그 자세로는 못 푸는 경우.
          ③ 관절 클램프(`RealBackend._clamp_arm`) — 최후 방어, 벤더 코드에 이미 있음.
        """
        if self._estopped:
            return

        verdict = self.check_workspace(xyz)                  # ① 실측 영역
        if verdict is not None and not verdict.ok:
            raise OutOfReach(verdict.reason)

        try:                                                 # ② IK 잔차
            angles_deg, _err = self.arm.go(list(xyz), real=self.real, down=down)
        except ValueError as exc:
            raise OutOfReach(str(exc)) from exc
        self._last_deg = list(angles_deg)
        if wait:
            self.wait_settled(angles_deg)

    def wait_settled(self, target_deg, timeout: float | None = None) -> bool:
        """목표각에 도달할 때까지 대기. 도달했으면 True, 시간초과면 False.

        백엔드가 둘 다 논블로킹이라(명령만 걸고 즉시 반환) 이 대기가 없으면
        팔이 아직 가는 중인데 다음 동작이 겹쳐 들어온다.
        """
        timeout = self.conf.settle_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        target = np.asarray(target_deg[:5], dtype=float)

        while time.monotonic() < deadline:
            if self._estopped:
                return False
            if self.should_abort is not None and self.should_abort():
                self.estop()          # 이동 한복판에서도 즉시 선다
                return False
            if self.real and self._servos_moving() is False:
                return True
            actual = self.read_arm_deg()
            if actual is not None:
                if np.max(np.abs(np.asarray(actual) - target)) <= self.conf.settle_tol_deg:
                    return True
            time.sleep(0.02)
        return False

    def _servos_moving(self) -> bool | None:
        """MOVING 레지스터(66)로 이동 중인지 확인. 못 읽으면 None → 위치 폴링 사용."""
        try:
            for sid in ARM_IDS:
                data = self.drv._read(sid, ADDR_MOVING, 1)
                if not data:
                    return None
                if data[0]:
                    return True
            return False
        except Exception:
            return None

    def home(self) -> None:
        """안전 대기 자세로. 시작·종료 때 팔을 예측 가능한 위치에 둔다."""
        if self._estopped:
            return
        self.set_grip(self.conf.grip_open, settle=0.2)
        deg = list(self.conf.home_pose_deg)
        self.backend.move(deg, grip=self.grip_map.to_deg(self._grip_frac))
        self._last_deg = deg
        self.wait_settled(deg)

    # ── 안전 ──────────────────────────────────────────────────────────────
    def estop(self) -> None:
        """즉시 정지 — **토크는 끄지 않는다.**

        토크를 끄면 팔이 중력에 주저앉는다. 사람 손이 근처에 있어서 멈추는
        상황에서 팔이 떨어지는 건 안 멈추느니만 못하다. 그래서 지금 위치를
        읽어 그 자리를 목표로 다시 걸어 그대로 굳힌다.
        """
        self._estopped = True
        try:
            if self.real:
                pos = self.drv.get_all_positions()
                for sid in ARM_IDS:
                    if pos.get(sid) is not None:
                        self.drv.set_position(sid, pos[sid])
            else:
                with self._sim_data():
                    self.backend.data.ctrl[:5] = self.backend.data.qpos[:5]
        except Exception as exc:                     # 정지는 실패해도 조용히 죽지 않는다
            print("[robot] E-STOP 중 오류:", exc)

    def release_estop(self) -> None:
        self._estopped = False

    @property
    def estopped(self) -> bool:
        return self._estopped

    # ── 수동 조깅 ─────────────────────────────────────────────────────────
    def jog(self, joint_index: int, delta_deg: float) -> None:
        """관절 하나를 delta 만큼 돌린다(0~4=팔, 5=그리퍼). 수동 모드 전용."""
        if self._estopped:
            return
        if joint_index == 5:
            self.set_grip(self._grip_frac + delta_deg, settle=0.0)
            return
        deg = list(self._last_deg)
        deg[joint_index] += delta_deg
        self.backend.move(deg, grip=self.grip_map.to_deg(self._grip_frac))
        self._last_deg = deg

    # ── 파지/놓기 시퀀스 ──────────────────────────────────────────────────
    def pick(self, ball_xy, attempt: int = 0) -> None:
        """공 옆으로 내려가 붙인 뒤 닫는다.

        공 '위'가 아니라 '옆'으로 가는 이유: 이 그리퍼는 한쪽 집게가 고정이라
        위에서 덮으면 공을 밀어낸다. soarm_lab.grasp.approach_xy 가 공에서
        베이스 쪽으로 비켜난 접근점을 계산해 준다.
        """
        from soarm_lab.grasp import approach_xy

        r = self.conf.ball_radius
        hx, hy = approach_xy(ball_xy, r=r, margin=0.014)   # 여유 있게 옆
        gx, gy = approach_xy(ball_xy, r=r, margin=0.0)     # 공에 붙임
        z = max(0.0, self.conf.z_grasp - attempt * self.conf.retry_z_drop)

        self.set_grip(self.conf.grip_open, settle=0.2)
        self.move_to([hx, hy, self.conf.z_hover], down=True)
        self.move_to([hx, hy, z], down=True)
        self.move_to([gx, gy, z], down=True)
        self.set_grip(self.conf.grip_closed, settle=0.5)
        self.move_to([gx, gy, self.conf.z_hover], down=True)

    def place(self, bin_xy) -> None:
        """bin 위로 옮겨 놓고 손을 뗀다. bin 좌표는 부를 때마다 새로 받는다."""
        bx, by = float(bin_xy[0]), float(bin_xy[1])
        self.move_to([bx, by, self.conf.z_hover], down=False)
        self.move_to([bx, by, self.conf.z_release], down=False)
        self.set_grip(self.conf.grip_open, settle=0.4)
        self.move_to([bx, by, self.conf.z_hover], down=False)

    def celebrate(self) -> None:
        """분류 완료 세리머니 — 그리퍼를 두 번 여닫고 좌우로 흔든다."""
        if self._estopped:
            return
        base = list(self._last_deg)
        for _ in range(2):
            self.set_grip(self.conf.grip_closed, settle=0.15)
            self.set_grip(self.conf.grip_open, settle=0.15)
        for delta in (25, -25, 25, -25, 0):
            deg = list(base)
            deg[0] += delta
            self.backend.move(deg, grip=self.grip_map.to_deg(self._grip_frac))
            self._last_deg = deg
            self.wait_settled(deg, timeout=1.2)

    def close(self) -> None:
        """종료 정리. 토크는 켜둔 채 둔다 — 끄면 팔이 그 자리에서 주저앉는다."""
        self._estopped = True
