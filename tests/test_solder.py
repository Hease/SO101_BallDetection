# -*- coding: utf-8 -*-
"""납땜 작업 검증 — 하드웨어 없이.

이 작업에는 공 분류에 없던 **도메인 규칙 두 가지**가 있고, 둘 다 말이 아니라
코드로 강제돼야 한다.

  1. 수평 이동은 반드시 안전 높이에서. 인두 끝을 옆으로 끌면 보드를 긁는다.
  2. 체류(2초)는 중간에 끊길 수 있어야 한다. `time.sleep(2)` 로 두면 그동안
     사람이 손을 넣어도 아무것도 감시하지 않는 구간이 생긴다.
"""
from __future__ import annotations

import threading


from sorting import config as cfg
from sorting.solder import Observation, SolderPipeline, State
from sorting.stats import SessionStats
from sorting.vision import Ball, Region, Scene
from sorting.watchdog import CAMERA, LatencyWatchdog, Thresholds

from .fakes import FakeRobot, fake_mapper

FAST = cfg.SolderConfig(dwell_s=0.05)      # 테스트는 체류를 짧게


class FakeBoard:
    """납땜점 마커가 붙은 가짜 보드."""

    def __init__(self, points=(), board_bbox=(40, 40, 560, 400)):
        self.points = [tuple(p) for p in points]      # [(color, u, v)]
        self.board_bbox = board_bbox
        self.intruded = False

    def observe(self) -> Observation:
        x, y, w, h = self.board_bbox
        regions = {cfg.SOLDER.board_color:
                   Region(cfg.SOLDER.board_color, x + w // 2, y + h // 2,
                          (x, y, w, h), float(w * h))}
        markers = [Ball(c, u, v, 8, 200.0) for c, u, v in self.points]
        return Observation(scene=Scene(balls=markers, regions=regions),
                           balls=markers, intruded=self.intruded)


def run(pipe: SolderPipeline, timeout: float = 10.0) -> threading.Thread:
    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    t.join(timeout)
    return t


# ── 기본 흐름 ──────────────────────────────────────────────────────────────
def test_solders_every_point_then_finishes():
    board = FakeBoard(points=[("red", 200, 200), ("red", 400, 300)])
    robot = FakeRobot()
    stats = SessionStats()
    events = []
    pipe = SolderPipeline(robot, fake_mapper(), board.observe, stats=stats,
                          conf=FAST, on_event=lambda n, d: events.append((n, d)))
    t = run(pipe)

    assert not t.is_alive()
    assert pipe.state is State.DONE
    assert stats.total == 2, f"납땜점 2개인데 {stats.total}개 처리"
    assert sum(1 for n, _ in events if n == "soldered") == 2
    assert any(n == "complete" for n, _ in events)


def test_each_point_is_soldered_once():
    """마커는 납땜 후에도 그대로 있다 — 우리가 기억하지 않으면 무한 반복된다."""
    board = FakeBoard(points=[("red", 250, 250)])
    robot = FakeRobot()
    stats = SessionStats()
    pipe = SolderPipeline(robot, fake_mapper(), board.observe, stats=stats,
                          conf=FAST)
    run(pipe)

    assert stats.total == 1, "같은 점을 여러 번 납땜했다"


# ── 도메인 규칙 ① 수평 이동은 안전 높이에서 ────────────────────────────────
def test_lateral_moves_only_at_safe_height():
    """★ 인두를 옆으로 끌지 않는가.

    보드를 긁고 부품을 밀어내는 실패 모드다. 좌표가 바뀌는(=수평으로 움직이는)
    이동은 전부 안전 높이여야 한다.
    """
    board = FakeBoard(points=[("red", 200, 200), ("red", 430, 320)])
    robot = FakeRobot()
    pipe = SolderPipeline(robot, fake_mapper(), board.observe, conf=FAST)
    run(pipe)

    moves = [c[1] for c in robot.calls if c[0] == "move_to"]
    assert len(moves) >= 6, "이동이 너무 적다 — 시나리오가 안 돌았다"

    prev = None
    for xyz in moves:
        if prev is not None:
            moved_sideways = (abs(xyz[0] - prev[0]) > 1e-6 or
                              abs(xyz[1] - prev[1]) > 1e-6)
            if moved_sideways:
                assert xyz[2] >= FAST.z_safe - 1e-9, (
                    f"인두를 낮은 높이({xyz[2]:.3f}m)에서 옆으로 끌었다 — "
                    f"안전 높이는 {FAST.z_safe}m")
        prev = xyz


def test_descend_and_retract_are_vertical():
    """하강·상승은 제자리에서 수직으로만 — xy 가 같아야 한다."""
    board = FakeBoard(points=[("red", 300, 250)])
    robot = FakeRobot()
    pipe = SolderPipeline(robot, fake_mapper(), board.observe, conf=FAST)
    run(pipe)

    moves = [c[1] for c in robot.calls if c[0] == "move_to"]
    low = [m for m in moves if m[2] < FAST.z_safe]
    assert low, "인두를 내린 적이 없다"
    for point in low:
        same_xy = [m for m in moves
                   if abs(m[0] - point[0]) < 1e-9 and abs(m[1] - point[1]) < 1e-9]
        assert any(m[2] >= FAST.z_safe for m in same_xy), \
            "내려간 자리에서 다시 올라오지 않았다"


# ── 도메인 규칙 ② 체류는 끊길 수 있어야 한다 ──────────────────────────────
def test_dwell_is_interruptible_by_intrusion():
    """★ 체류 중 사람이 손을 넣으면 2초를 다 기다리지 않는다."""
    board = FakeBoard(points=[("red", 300, 250)])
    robot = FakeRobot()
    slow = cfg.SolderConfig(dwell_s=30.0)          # 아주 긴 체류
    pipe = SolderPipeline(robot, fake_mapper(), board.observe, conf=slow)

    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()

    # 체류에 들어갈 때까지 기다린다
    for _ in range(100):
        if pipe.state is State.DWELL:
            break
        threading.Event().wait(0.02)
    assert pipe.state is State.DWELL, "체류 상태에 못 들어갔다"

    board.intruded = True                           # 손이 들어온다
    for _ in range(100):
        if pipe.state is not State.DWELL:
            break
        threading.Event().wait(0.02)

    assert pipe.state is not State.DWELL, \
        "손이 들어왔는데 30초 체류를 그대로 기다리고 있다"
    pipe.request_stop()
    board.intruded = False
    t.join(5.0)


def test_interrupted_point_is_retried_not_marked_done():
    """중단된 점은 '했다'로 치지 않는다 — 덜 녹은 납땜을 남기면 안 된다."""
    board = FakeBoard(points=[("red", 300, 250)])
    robot = FakeRobot()
    stats = SessionStats()
    slow = cfg.SolderConfig(dwell_s=20.0)
    pipe = SolderPipeline(robot, fake_mapper(), board.observe, conf=slow,
                          stats=stats)

    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    for _ in range(100):
        if pipe.state is State.DWELL:
            break
        threading.Event().wait(0.02)

    board.intruded = True
    threading.Event().wait(0.3)
    pipe.request_stop()
    board.intruded = False
    t.join(5.0)

    assert stats.total == 0, "중단됐는데 완료로 셌다"
    assert not pipe._done, "중단된 점이 '완료' 목록에 들어갔다"


# ── 안전장치가 그대로 붙는가 ───────────────────────────────────────────────
def test_watchdog_stop_reaches_robot():
    """딜레이 워치독이 납땜 작업에도 같은 통로로 붙는가."""
    board = FakeBoard(points=[("red", 300, 250)])
    robot = FakeRobot()
    wd = LatencyWatchdog(Thresholds(warn_ms=300, hold_ms=800, stop_ms=1500,
                                    recover_samples=2))
    SolderPipeline(robot, fake_mapper(), board.observe, conf=FAST, watchdog=wd)

    assert robot.should_abort is not None
    assert robot.should_abort() is False
    wd.report(CAMERA, 2000)
    assert robot.should_abort() is True


def test_point_outside_workspace_is_skipped_not_fatal():
    """작업영역 밖의 점 하나 때문에 전체가 죽으면 안 된다."""
    # 보드를 화면 전체로 잡는다 — 안 그러면 먼 마커가 '보드 밖'으로 먼저
    # 걸러져서, 정작 확인하려던 도달한계 경로를 타지 않는다.
    board = FakeBoard(points=[("red", 620, 470), ("red", 300, 250)],
                      board_bbox=(0, 0, 640, 480))
    robot = FakeRobot(reach_limit=0.28)
    stats = SessionStats()
    pipe = SolderPipeline(robot, fake_mapper(), board.observe, conf=FAST,
                          stats=stats)
    t = run(pipe)

    assert not t.is_alive()
    assert pipe.state is State.DONE
    assert stats.unreachable >= 1
    assert stats.total >= 1, "닿는 점은 납땜했어야 한다"


def test_intrusion_pauses_and_keeps_finished_points():
    """재개할 때 이미 한 점을 다시 납땜하면 안 된다.

    공 분류에서는 사람이 공을 옮겼을 수 있어 포기 목록을 비웠지만, 납땜은
    '이미 했다'가 물리적으로 되돌릴 수 없는 사실이다.
    """
    board = FakeBoard(points=[("red", 200, 200), ("red", 420, 320)])
    robot = FakeRobot()
    stats = SessionStats()
    pipe = SolderPipeline(robot, fake_mapper(), board.observe, conf=FAST,
                          stats=stats)

    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    for _ in range(200):                       # 한 점이라도 끝나길 기다린다
        if len(pipe._done) >= 1:
            break
        threading.Event().wait(0.01)

    board.intruded = True
    threading.Event().wait(0.2)
    assert pipe.state is State.PAUSED
    board.intruded = False
    t.join(10.0)

    assert not t.is_alive()
    assert stats.total == 2, f"재개 후 총 2점이어야 하는데 {stats.total}점"


# ── 보드 밖은 무시한다 ─────────────────────────────────────────────────────
def test_markers_outside_the_board_are_ignored():
    """책상에 굴러다니는 같은 색 물건을 납땜하러 가지 않는다."""
    board = FakeBoard(points=[("red", 300, 250), ("red", 10, 10)],
                      board_bbox=(100, 100, 400, 300))
    robot = FakeRobot()
    stats = SessionStats()
    pipe = SolderPipeline(robot, fake_mapper(), board.observe, conf=FAST,
                          stats=stats)
    run(pipe)

    assert stats.total == 1, "보드 밖 마커까지 납땜했다"


def test_no_points_finishes_immediately():
    board = FakeBoard(points=[])
    robot = FakeRobot()
    pipe = SolderPipeline(robot, fake_mapper(), board.observe, conf=FAST)
    t = run(pipe, timeout=5.0)

    assert not t.is_alive()
    assert pipe.state is State.DONE
    assert "home" in robot.names_called(), "끝나면 안전 자세로 돌아가야 한다"
