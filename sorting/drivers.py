# -*- coding: utf-8 -*-
"""drivers.py — 하드웨어를 **이름으로 갈아끼운다.**

    python sorting_main.py --robot print --camera replay

새 로봇이나 새 카메라가 들어와도 위쪽 코드(`pipeline`·`vision`·`workspace`·
`watchdog`)는 한 줄도 안 바뀐다. 여기에 어댑터 하나를 추가하고 등록만 하면 된다.

등록된 것들:

    로봇    so101   실제 SO-ARM101 (시리얼 서보) 또는 헤드리스 MuJoCo 시뮬
            print   **아무것도 안 움직이고 무엇을 할지 print 만 한다**
    카메라  hp60c   실제 HP60C 뎁스카메라
            replay  저장된 사진을 카메라인 척 재생
            print   합성 화면을 만들어 내보내며 무엇을 봤는지 print

`print` 드라이버가 있는 이유는 두 가지다. 하나는 하드웨어가 없어도 전체 흐름을
끝까지 돌려볼 수 있다는 것이고, 다른 하나는 **계층이 정말로 분리돼 있는지에 대한
증명**이다. 로봇을 print 로 바꿔도 상태머신·안전장치·GUI 가 그대로 돌면, 그
경계가 진짜라는 뜻이다.

새 하드웨어를 붙이는 법은 `docs/DEV.md` 의 "하드웨어가 바뀌면" 절을 보라.
"""
from __future__ import annotations

import time

import numpy as np

from . import config as cfg
from .ports import CameraPort, RobotPort, missing_methods


# ══════════════════════════════════════════════════════════════════════════
#  로봇 드라이버
# ══════════════════════════════════════════════════════════════════════════
class PrintRobot:
    """움직이지 않고 **무엇을 할지 말만 하는** 로봇.

    하드웨어가 없거나, 고장났거나, 아직 안 왔을 때 쓴다. 좌표 계산·안전 판정·
    상태머신은 전부 진짜와 똑같이 돌고 마지막 '팔을 움직이는' 한 단계만
    출력으로 바뀐다.

    가짜지만 **거짓말은 하지 않는다**: 자기 자세를 기억하고, 그리퍼 개도에 따라
    파지 성공 여부를 답하고, E-STOP 을 걸면 실제로 이후 명령을 무시한다.
    그래야 위쪽 코드가 진짜 로봇에서와 같은 경로를 탄다.
    """

    describe = "print 로봇 (동작 없음 — 명령만 출력)"

    def __init__(self, conf: cfg.RobotConfig | None = None, verbose: bool = True,
                 grip_closed_means_holding: bool = True):
        self.conf = conf or cfg.ROBOT
        self.verbose = verbose
        self.should_abort = None
        self.workspace = None

        self._estopped = False
        self._grip_pct = self.conf.grip_open
        self._xyz = (0.0, 0.0, 0.0)
        self._joints = list(self.conf.home_pose_deg)
        self._holding_when_closed = grip_closed_means_holding
        self.log: list[str] = []      # 테스트가 들여다볼 수 있게 남긴다

        from .workspace import Workspace
        self.workspace = Workspace.load()

    # ── 출력 ──────────────────────────────────────────────────────────────
    def _say(self, text: str) -> None:
        self.log.append(text)
        if self.verbose:
            print(f"[로봇] {text}")

    # ── 상태 ──────────────────────────────────────────────────────────────
    @property
    def estopped(self) -> bool:
        return self._estopped

    def joint_angles_deg(self) -> list[float]:
        return list(self._joints) + [self._grip_pct]

    def read_arm_deg(self) -> list[float]:
        return list(self._joints)

    def check_workspace(self, xyz):
        """진짜 로봇과 같은 검사를 한다 — 한계 데모가 print 모드에서도 돈다."""
        if self.workspace is None:
            return None
        return self.workspace.check(float(xyz[0]), float(xyz[1]), float(xyz[2]))

    # ── 동작 ──────────────────────────────────────────────────────────────
    def home(self) -> None:
        if self._estopped:
            return
        self._joints = list(self.conf.home_pose_deg)
        self._say(f"홈 자세로 이동 {[round(d) for d in self._joints]}")

    def move_to(self, xyz, down: bool = False, wait: bool = True) -> None:
        if self._estopped:
            return
        verdict = self.check_workspace(xyz)
        if verdict is not None and not verdict.ok:
            from .robot import OutOfReach
            raise OutOfReach(verdict.reason)

        self._xyz = tuple(float(v) for v in xyz)
        # 자세가 실제로 변하는 것처럼 보이게 — 3D 트윈이 멈춰 있지 않도록
        self._joints[0] = float(np.degrees(np.arctan2(self._xyz[1], self._xyz[0])))
        self._say(f"이동 → ({self._xyz[0]:+.3f}, {self._xyz[1]:+.3f}, "
                  f"{self._xyz[2]:+.3f})m" + ("  [아래보기]" if down else ""))

    def pick(self, target_xy, attempt: int = 0) -> None:
        if self._estopped:
            return
        retry = f" (재시도 {attempt})" if attempt else ""
        self._say(f"집기{retry} @ ({target_xy[0]:+.3f}, {target_xy[1]:+.3f})")
        self.move_to((target_xy[0], target_xy[1], self.conf.z_hover), down=True)
        self.set_grip(self.conf.grip_open)
        self.move_to((target_xy[0], target_xy[1], self.conf.z_grasp), down=True)
        self.set_grip(self.conf.grip_closed)
        self.move_to((target_xy[0], target_xy[1], self.conf.z_hover), down=True)

    def place(self, target_xy) -> None:
        if self._estopped:
            return
        self._say(f"놓기 @ ({target_xy[0]:+.3f}, {target_xy[1]:+.3f})")
        self.move_to((target_xy[0], target_xy[1], self.conf.z_hover))
        self.move_to((target_xy[0], target_xy[1], self.conf.z_release))
        self.set_grip(self.conf.grip_open)
        self.move_to((target_xy[0], target_xy[1], self.conf.z_hover))

    def set_grip(self, pct: float, settle: float = 0.0) -> None:
        if self._estopped:
            return
        self._grip_pct = max(0.0, min(100.0, float(pct)))
        self._say(f"그리퍼 {self._grip_pct:.0f}%")

    def holding_object(self) -> bool:
        """닫혀 있으면 물었다고 본다 — 진짜 로봇의 판정 규칙과 같은 모양."""
        if not self._holding_when_closed:
            return False
        return self._grip_pct > self.conf.grip_empty_pct

    def jog(self, joint_index: int, delta_deg: float) -> None:
        if self._estopped:
            return
        if joint_index == 5:
            self.set_grip(self._grip_pct + delta_deg)
            return
        self._joints[joint_index] += delta_deg
        self._say(f"조깅 J{joint_index + 1} {delta_deg:+.0f}° "
                  f"→ {self._joints[joint_index]:.0f}°")

    # ── 안전 ──────────────────────────────────────────────────────────────
    def estop(self) -> None:
        self._estopped = True
        self._say("■ 비상정지 — 현재 자세 유지 (토크 안 끔)")

    def release_estop(self) -> None:
        self._estopped = False
        self._say("정지 해제")

    def close(self) -> None:
        self._say("종료")


def _make_so101(real: bool = False, slow: bool = False, **_kw):
    from .robot import RobotController
    return RobotController(real=real, slow=slow)


def _make_print_robot(**kw):
    return PrintRobot(verbose=kw.get("verbose", True))


# ══════════════════════════════════════════════════════════════════════════
#  카메라 드라이버
# ══════════════════════════════════════════════════════════════════════════
class PrintCamera:
    """카메라 없이 **장면을 합성해** 내보내고, 무엇을 만들었는지 print 한다.

    검출기가 실제로 찾아낼 수 있는 색 동그라미와 사각형을 그린다. 그래서
    비전 → 좌표변환 → 상태머신까지 전 구간이 진짜와 같은 경로로 돈다.
    파이프라인이 집어간 물체를 지워 달라고 `remove_at()` 을 부를 수도 있다.
    """

    describe = "print 카메라 (합성 화면 — 하드웨어 없음)"

    def __init__(self, width: int = 640, height: int = 480, fps: float = 20.0,
                 verbose: bool = True):
        self.width, self.height = width, height
        self.period = 1.0 / max(fps, 0.1)
        self.verbose = verbose
        self._frame_id = 0
        self.log: list[str] = []

        # 화면에 놓을 것들: (색이름, u, v, 반지름). 검출기가 이걸 찾아낸다.
        self.balls = [("red", 250, 300, 20), ("blue", 390, 300, 20)]
        self.bins = {"red": (110, 120), "blue": (530, 120)}

    def _say(self, text: str) -> None:
        self.log.append(text)
        if self.verbose:
            print(f"[카메라] {text}")

    def remove_at(self, u: int, v: int, tol: int = 40) -> bool:
        """그 자리의 물체를 치운다 — 로봇이 집어간 것을 반영."""
        for i, (_c, bu, bv, _r) in enumerate(self.balls):
            if abs(bu - u) <= tol and abs(bv - v) <= tol:
                self.balls.pop(i)
                return True
        return False

    def _render(self) -> np.ndarray:
        import cv2
        from . import config as c

        frame = np.full((self.height, self.width, 3), 230, dtype=np.uint8)

        def bgr_of(name: str):
            """Lab 기준색을 실제 BGR 픽셀로 되돌린다 — 검출기가 찾을 수 있게."""
            band = {"red": c.RED, "blue": c.BLUE, "orange": c.ORANGE}[name]
            lab = np.uint8([[[160, int(band.a) + 128, int(band.b) + 128]]])
            return tuple(int(v) for v in cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)[0, 0])

        for name, (u, v) in self.bins.items():
            cv2.rectangle(frame, (u - 55, v - 45), (u + 55, v + 45),
                          bgr_of(name), -1)
        for name, u, v, r in self.balls:
            cv2.circle(frame, (u, v), r, bgr_of(name), -1)
        return frame

    def read_blocking(self, last_frame_id: int = 0, timeout: float = 1.0,
                      poll_hz: float = 200.0):
        time.sleep(self.period)
        self._frame_id = last_frame_id + 1
        if self._frame_id % 20 == 1:      # 매 프레임 찍으면 시끄럽다
            self._say(f"프레임 {self._frame_id} 합성 — 공 {len(self.balls)}개, "
                      f"구역 {len(self.bins)}개 (깊이 없음)")
        return self._render(), None, self._frame_id

    def __enter__(self) -> "PrintCamera":
        self._say("열림 (합성 모드)")
        return self

    def __exit__(self, *_exc) -> bool:
        self._say("닫힘")
        return False


def _make_hp60c(**_kw):
    from hp60c_camera import CameraReader
    return CameraReader()


def _make_replay(path: str = "shots", **kw):
    from .replay import ReplaySource
    return ReplaySource(path, fps=kw.get("fps", 10.0))


def _make_print_camera(**kw):
    return PrintCamera(verbose=kw.get("verbose", True))


# ══════════════════════════════════════════════════════════════════════════
#  등록소
# ══════════════════════════════════════════════════════════════════════════
ROBOTS = {
    "so101": _make_so101,      # 실제 팔 또는 헤드리스 시뮬
    "print": _make_print_robot,
}

CAMERAS = {
    "hp60c": _make_hp60c,
    "replay": _make_replay,
    "print": _make_print_camera,
}


class UnknownDriver(SystemExit):
    """이름을 잘못 줬을 때. 무엇을 쓸 수 있는지 같이 알려준다."""


def make_robot(name: str = "so101", **kw):
    if name not in ROBOTS:
        raise UnknownDriver(
            f"모르는 로봇 드라이버 '{name}'. 쓸 수 있는 것: {', '.join(ROBOTS)}")
    return ROBOTS[name](**kw)


def make_camera(name: str = "hp60c", **kw):
    if name not in CAMERAS:
        raise UnknownDriver(
            f"모르는 카메라 드라이버 '{name}'. 쓸 수 있는 것: {', '.join(CAMERAS)}")
    return CAMERAS[name](**kw)


def check(obj, port) -> list[str]:
    """이 물건이 계약을 지키는가. 빠진 메서드 목록을 돌려준다(비어 있으면 합격)."""
    return missing_methods(obj, port)


# ── 자기 점검 CLI ──────────────────────────────────────────────────────────
def _selfcheck() -> int:
    """등록된 드라이버가 계약을 지키는지 확인한다. 새 드라이버를 붙인 뒤 실행."""
    ok = True
    print("\n등록된 드라이버 계약 점검\n")
    for label, table, port, kw in (("로봇", ROBOTS, RobotPort, {"verbose": False}),
                                   ("카메라", CAMERAS, CameraPort, {"verbose": False})):
        for name, factory in table.items():
            try:
                obj = factory(**kw)
            except Exception as exc:                 # 하드웨어가 없으면 못 만든다
                print(f"  ⏭ {label} {name:<8} 생성 불가(하드웨어 없음): "
                      f"{type(exc).__name__}")
                continue
            gaps = check(obj, port)
            mark = "✅" if not gaps else "❌"
            print(f"  {mark} {label} {name:<8} {getattr(obj, 'describe', '')}")
            if gaps:
                print(f"       빠진 것: {', '.join(gaps)}")
                ok = False
            if hasattr(obj, "close"):
                try:
                    obj.close()
                except Exception:
                    pass
    print()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_selfcheck())
