# -*- coding: utf-8 -*-
"""safety.py — 납땜 모사 실습 공용 '안전 골격'.

브리프 §8·§10 의 안전 요구사항을 한 곳에 모은다. 동작 스크립트(10_teach·11_replay·
이후 비주얼 서보잉)는 전부 이걸 import 해서 매 iteration 마다 검사한다.
나중에 붙이면 못 붙이므로(§243) 골격을 먼저 깐다.

담는 것:
  EStop        긴급정지 — Enter 또는 Ctrl-C → 정지 플래그 + 토크오프 콜백
  LoadGuard    서보 부하 감시 — 접촉(급증) 판정 겸 과부하 즉시정지 (§6, §8)
  FrameWatchdog 카메라 프레임 정지/지연 감지 — N ms 끊기면 동작 정지 (§8)
  관절/이동 한계 — check_joint_limits · clamp_deg5 · clamp_delta (§8, §10.5)

숫자(한계값)는 전부 이 파일 상단에 둔다(§10.4). 현장에서 실측해 바꾼다.
"""
import os
import select
import sys
import threading
import time

import numpy as np

# fk_core 는 soarm_lab/ 안에 있다. mock 모드(soarm_lab 미import)에서도 관절한계를
# 읽을 수 있게, 이 파일이 스스로 경로를 등록한다(자기완결).
_SOARM = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "soarm_lab")
if _SOARM not in sys.path:
    sys.path.insert(0, _SOARM)
from fk_core import FKSo101, ARM_JOINT_NAMES

# ── 한계값 (현장 실측으로 조정) ───────────────────────────────────────────────
MAX_STEP_DEG = 8.0        # 한 명령당 관절 최대 이동(도) — delta clamp (§10.5)
LOAD_CONTACT = 250        # 이 부하(절대값) 이상이면 '접촉'으로 판정 — 실측 필수
LOAD_ABORT = 700          # 이 부하 이상이면 과부하 → 즉시정지
FRAME_STALL_MS = 300      # 프레임이 이 시간 이상 안 바뀌면 카메라 정지로 판정
LOAD_IDS = (2, 3)         # 수직 반력을 주로 받는 관절(shoulder_lift·elbow_flex)


class Stopped(Exception):
    """긴급정지/한계초과로 동작을 중단할 때 던진다."""


# ── 긴급정지 ─────────────────────────────────────────────────────────────────
class EStop:
    """긴급정지 버튼(§8). 백그라운드로 표준입력을 지켜보다 Enter 가 오면 정지.
    Ctrl-C(SIGINT) 로도 trip() 을 부를 수 있다. 정지 시 on_stop 을 한 번 호출.

        est = EStop(on_stop=lambda: robot.estop()).start()
        while ...:
            est.check()          # 정지됐으면 Stopped 예외

    on_stop 은 `robot.estop()` 이다 — **토크를 끄지 않는다.** 끄면 중력으로
    주저앉고, 인두기를 물고 있으면 그게 작업면으로 떨어진다. 현재 자세를 다시
    목표로 걸어 그 자리에 굳히는 쪽이 안전하다.
    """

    def __init__(self, on_stop=None):
        self._stop = threading.Event()
        self._fired = False
        self.on_stop = on_stop
        self._t = threading.Thread(target=self._watch_stdin, daemon=True)

    def start(self):
        print("[E-STOP] 준비됨 — 멈추려면 Enter (또는 Ctrl-C).")
        self._t.start()
        return self

    def _watch_stdin(self):
        while not self._stop.is_set():
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0.2)
            except (ValueError, OSError):
                return                       # stdin 닫힘
            if r:
                # EOF 는 사람이 Enter 를 누른 게 아니다. 파이프·리다이렉션으로
                # 돌리면 select 가 즉시 readable 을 주고 readline 이 "" 를
                # 돌려주는데, 이걸 입력으로 세면 시작하자마자 정지해 버린다.
                if sys.stdin.readline() == "":
                    print("[E-STOP] stdin EOF — 키보드 정지는 쓸 수 없습니다"
                          " (Ctrl-C 는 유효)")
                    return
                self.trip("Enter 입력")

    def trip(self, why="?"):
        """정지시킨다(중복 호출 안전). on_stop 콜백을 한 번만 실행."""
        self._stop.set()
        if not self._fired:
            self._fired = True
            print(f"\n[E-STOP] {why} → 정지 (현재 자세 유지 · 토크 안 끔)")
            if self.on_stop:
                try:
                    self.on_stop()
                except Exception as e:       # 콜백 실패해도 정지 자체는 유지
                    print("  on_stop 오류:", e)

    @property
    def stopped(self):
        return self._stop.is_set()

    def check(self):
        """정지 상태면 Stopped 예외. 모든 루프의 매 iteration 첫 줄에 둔다."""
        if self._stop.is_set():
            raise Stopped("긴급정지")


# ── 관절/이동 한계 (§8, §10.5) ───────────────────────────────────────────────
_FK = None


def _joint_limits_deg():
    """5개 팔 관절의 (하한, 상한)[도]. real.py 와 같은 소스(FK URDF)에서 가져온다."""
    global _FK
    if _FK is None:
        _FK = FKSo101()
    lim = _FK.limits_deg()
    lo = [lim[n][0] for n in ARM_JOINT_NAMES]
    hi = [lim[n][1] for n in ARM_JOINT_NAMES]
    return lo, hi


def check_joint_limits(deg5):
    """한계를 넘는 관절 목록 반환. 빈 리스트면 통과.
    각 항목 = (관절id, 값, 하한, 상한)."""
    lo, hi = _joint_limits_deg()
    return [(i + 1, float(d), lo[i], hi[i])
            for i, d in enumerate(deg5) if d < lo[i] - 1e-6 or d > hi[i] + 1e-6]


def clamp_deg5(deg5):
    """5관절 각도를 한계 안으로 클램프."""
    lo, hi = _joint_limits_deg()
    return [float(np.clip(d, lo[i], hi[i])) for i, d in enumerate(deg5)]


def clamp_delta(cur5, tgt5, max_step=MAX_STEP_DEG):
    """한 스텝의 관절 이동량을 max_step(도)로 제한한다(§10.5).
    반환 (reached, next5): reached=이번에 목표 도달, next5=이번에 보낼 각도.
    큰 이동은 여러 스텝으로 쪼개 팔이 홱 튀지 않게 한다."""
    cur = np.asarray(cur5, float)
    tgt = np.asarray(tgt5, float)
    d = tgt - cur
    m = float(np.max(np.abs(d))) if d.size else 0.0
    if m <= max_step:
        return True, tgt.tolist()
    return False, (cur + d * (max_step / m)).tolist()


# ── 부하 감시: 접촉 감지 + 과부하 정지 (§6, §8) ──────────────────────────────
class LoadGuard:
    """서보 Present_Load 로 (1) 접촉 급증 판정 (2) 과부하 즉시정지.
    먼저 공회전 부하를 baseline() 으로 재두면 상대 급증으로 접촉을 잡는다."""

    def __init__(self, drv, ids=LOAD_IDS, contact=LOAD_CONTACT, abort=LOAD_ABORT):
        self.drv = drv
        self.ids = ids
        self.contact = contact
        self.abort = abort
        self.base = {i: 0 for i in ids}

    def read(self):
        """{id: |load|} 현재값."""
        out = {}
        for i in self.ids:
            v = self.drv.get_load(i)
            if v is not None:
                out[i] = abs(v)
        return out

    def peak_rise(self):
        """baseline 대비 최대 부하 상승량."""
        cur = self.read()
        return max((cur[i] - self.base.get(i, 0) for i in cur), default=0)

    def baseline(self, samples=5, dt=0.05):
        """공회전(비접촉) 상태에서 몇 번 읽어 기준 부하를 잡는다."""
        acc = {i: [] for i in self.ids}
        for _ in range(samples):
            for i, v in self.read().items():
                acc[i].append(v)
            time.sleep(dt)
        self.base = {i: (int(np.median(vs)) if vs else 0) for i, vs in acc.items()}
        return self.base

    def contacted(self):
        """접촉(부하 급증) 판정."""
        return self.peak_rise() >= self.contact

    def overloaded(self):
        """과부하(절대 한계 초과) — 즉시정지 대상."""
        cur = self.read()
        return max(cur.values(), default=0) >= self.abort


# ── 프레임 지연 워치독 (§8) ──────────────────────────────────────────────────
class FrameWatchdog:
    """카메라 프레임 정지/지연 감지. 매 프레임 update(frame_id) 호출.
    frame_id 가 stall_ms 이상 안 바뀌면 stalled()=True → 동작을 멈춘다.
    (카메라를 가리거나 케이블을 뽑아 데모 가능 — §8)"""

    def __init__(self, stall_ms=FRAME_STALL_MS):
        self.stall = stall_ms / 1000.0
        self.last_id = None
        self.last_wall = time.monotonic()

    def freshness(self, frame_id):
        """마지막으로 프레임이 바뀐 뒤 흐른 시간(초)."""
        now = time.monotonic()
        if frame_id != self.last_id:
            self.last_id = frame_id
            self.last_wall = now
        return now - self.last_wall

    def stalled(self, frame_id):
        return self.freshness(frame_id) > self.stall
