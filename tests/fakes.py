# -*- coding: utf-8 -*-
"""fakes.py — 하드웨어 없이 돌리기 위한 대역들. **세 트랙이 공유한다.**

이 파일이 있는 이유는 협업 때문이다. 캘리브레이션 담당·안전 담당·비전 담당이
동시에 작업하려면, 서로의 구현이 끝나기를 기다리지 않고 자기 테스트를 쓸 수
있어야 한다. 그래서 로봇·카메라·장면을 흉내내는 대역을 한곳에 모아둔다.

  FakeRobot  물리 없이 즉시 이동하는 로봇 — MuJoCo 를 안 띄우므로 매우 빠르다.
             실물/시뮬 테스트가 87초 걸리던 것을 밀리초로 줄인다.
  FakeWorld  공과 bin 이 있는 가짜 책상. 관측(Observation)을 만들어 준다.
  FakeMapper 픽셀↔로봇 좌표를 단순 선형으로 잇는 캘리브레이션 대역.
"""
from __future__ import annotations

import cv2
import numpy as np

from sorting.mapping import Mapper
from sorting.pipeline import Observation
from sorting.vision import Ball, Region, Scene


def fake_mapper() -> Mapper:
    """640x480 화면이 로봇 앞 x=0.15~0.30, y=-0.12~+0.12 에 대응한다고 본다."""
    src = np.float32([[0, 0], [640, 0], [640, 480], [0, 480]])
    dst = np.float32([[0.15, 0.12], [0.15, -0.12], [0.30, -0.12], [0.30, 0.12]])
    return Mapper(cv2.getPerspectiveTransform(src, dst))


class FakeRobot:
    """RobotController 와 같은 모양이지만 물리도 시리얼도 없다.

    이동은 즉시 끝난 것으로 친다. 파이프라인의 *순서와 판단*을 검증하는 데는
    이걸로 충분하고, 실제 IK·물리가 필요한 테스트만 진짜 로봇을 쓰면 된다.
    """

    def __init__(self, holding: bool = True, reach_limit: float | None = None):
        self.should_abort = None
        self.estopped = False
        self.grip_pct = 100.0
        self.pose = [0.0, 30.0, -45.0, 0.0, 0.0]

        self._holding = holding
        self._reach_limit = reach_limit      # 이 반경 밖이면 OutOfReach 를 낸다
        self.calls: list[tuple] = []         # 무엇을 어떤 순서로 불렀는지

    # ── 기록 ──────────────────────────────────────────────────────────────
    def _log(self, name: str, *args) -> None:
        self.calls.append((name, *args))

    def names_called(self) -> list[str]:
        return [c[0] for c in self.calls]

    # ── 동작 ──────────────────────────────────────────────────────────────
    def _check_reach(self, xy) -> None:
        if self._reach_limit is None:
            return
        if float(np.hypot(xy[0], xy[1])) > self._reach_limit:
            from sorting.robot import OutOfReach
            raise OutOfReach(f"가짜 한계 {self._reach_limit}m 초과")

    def home(self) -> None:
        self._log("home")

    def move_to(self, xyz, down: bool = False, wait: bool = True) -> None:
        self._check_reach(xyz)
        self._log("move_to", tuple(xyz))

    def pick(self, ball_xy, attempt: int = 0) -> None:
        self._check_reach(ball_xy)
        self._log("pick", tuple(ball_xy), attempt)

    def place(self, bin_xy) -> None:
        self._check_reach(bin_xy)
        self._log("place", tuple(bin_xy))

    def set_grip(self, pct: float, settle: float = 0.0) -> None:
        self.grip_pct = pct
        self._log("set_grip", pct)

    def holding_object(self) -> bool:
        return self._holding

    def set_holding(self, value: bool) -> None:
        self._holding = value

    def jog(self, joint_index: int, delta_deg: float) -> None:
        self._log("jog", joint_index, delta_deg)

    def joint_angles_deg(self) -> list[float]:
        return list(self.pose) + [self.grip_pct]

    def read_arm_deg(self):
        return list(self.pose)

    # ── 안전 ──────────────────────────────────────────────────────────────
    def estop(self) -> None:
        self.estopped = True
        self._log("estop")

    def release_estop(self) -> None:
        self.estopped = False
        self._log("release_estop")

    def close(self) -> None:
        self._log("close")


class FakeWorld:
    """공과 bin 이 있는 가짜 책상. 로봇이 놓으면 공이 사라진다."""

    def __init__(self, balls=(), bins=None, pick_zone: bool = True):
        self.balls = [tuple(b) for b in balls]       # [(color, u, v)]
        self.bins = dict(bins or {})                 # {color: (u, v)}
        self.pick_zone = pick_zone
        self.intruded = False
        self.observations = 0

    def move_bin(self, color: str, u: int, v: int) -> None:
        """데모의 핵심 — 돌아가는 중에 bin 을 옮긴다."""
        self.bins[color] = (u, v)

    def remove_ball(self, ball) -> bool:
        """로봇이 집어간 **그 공**을 치운다.

        목록의 0번을 지우면 안 된다 — 파이프라인은 베이스에 가까운 공부터
        고르므로 처리 순서와 목록 순서가 다르다. 엉뚱한 공을 지우면 실제로는
        처리되지 않은 공이 사라져 테스트가 조용히 틀린 것을 통과시킨다.
        """
        for i, (color, u, v) in enumerate(self.balls):
            if color == ball.color and u == ball.u and v == ball.v:
                self.balls.pop(i)
                return True
        return False

    def observe(self) -> Observation:
        self.observations += 1
        regions = {c: Region(c, u, v, (u - 60, v - 60, 120, 120), 14400.0)
                   for c, (u, v) in self.bins.items()}
        zone = (Region("orange", 320, 240, (0, 0, 640, 480), 307200.0)
                if self.pick_zone else None)
        balls = [Ball(c, u, v, 20, 1200.0) for c, u, v in self.balls]
        return Observation(scene=Scene(balls=balls, regions=regions, pick_zone=zone),
                           balls=balls, intruded=self.intruded)
