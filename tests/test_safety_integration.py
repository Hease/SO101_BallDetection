# -*- coding: utf-8 -*-
"""안전장치가 *실제로 연결돼 있는가* — 모듈 단위가 아니라 배선을 검증한다.

workspace 와 watchdog 이 각각 잘 도는 것과, 그것들이 파이프라인·로봇에 실제로
물려 있는 것은 다른 문제다. 배선이 끊겨 있으면 테스트는 다 통과하는데 로봇은
그냥 한계를 넘어간다. 이 파일이 그 틈을 막는다.
"""
from __future__ import annotations

import threading

import pytest

from sorting.pipeline import SortingPipeline, State
from sorting.watchdog import CAMERA, LatencyWatchdog, Level, Thresholds
from sorting.workspace import Workspace

from .fakes import FakeRobot, FakeWorld, fake_mapper

TH = Thresholds(warn_ms=300, hold_ms=800, stop_ms=1500, recover_samples=2)


# ── 도달 한계가 실제로 집행되는가 ──────────────────────────────────────────
def test_workspace_is_checked_before_ik():
    """★ IK 를 부르기 전에 영역이 먼저 거절해야 한다.

    IK 는 반경 0.44m 까지 잔차 0mm 로 푼다. 영역 검사가 IK 뒤에 있거나 아예
    연결돼 있지 않으면 그 좌표가 그대로 통과한다.
    """
    from sorting.robot import OutOfReach, RobotController

    ws = Workspace.from_trace([(0.12, -0.12, 0.0), (0.26, -0.12, 0.0),
                               (0.26, 0.12, 0.2), (0.12, 0.12, 0.0)])

    # RobotController 를 통째로 만들면 MuJoCo 가 뜬다. 검사 순서만 보면 되므로
    # move_to 가 실제로 건드리는 것만 갖춘 최소 스텁에 진짜 메서드를 붙인다.
    class Stub:
        _estopped = False
        real = False
        workspace = ws
        ik_called = False
        _last_deg = [0, 0, 0, 0, 0]

        class arm:
            @staticmethod
            def go(*_a, **_k):
                Stub.ik_called = True
                return [0, 0, 0, 0, 0], 0.0

        def wait_settled(self, *_a, **_k):
            return True

        check_workspace = RobotController.check_workspace
        move_to = RobotController.move_to

    stub = Stub()
    assert not stub.check_workspace((0.44, 0.0, 0.1)).ok

    with pytest.raises(OutOfReach):
        stub.move_to((0.44, 0.0, 0.1))
    assert not Stub.ik_called, "영역 밖인데 IK 를 불렀다 — 검사가 IK 뒤에 있다"

    stub.move_to((0.20, 0.0, 0.1))               # 영역 안은 통과해 IK 로 간다
    assert Stub.ik_called


def test_workspace_absent_means_no_pretend_limit():
    """아직 안 쟀으면 '한계가 있는 척' 하지 않는다."""
    from sorting.robot import RobotController

    class Stub:
        workspace = None
        check_workspace = RobotController.check_workspace

    assert Stub().check_workspace((0.44, 0.0, 0.1)) is None


def test_unreachable_ball_is_skipped_and_counted():
    """영역 밖 공은 건너뛰되 사이클은 계속되고, 통계에 남는다."""
    from sorting.stats import SessionStats

    world = FakeWorld(balls=[("red", 620, 460), ("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = FakeRobot(reach_limit=0.28)
    stats = SessionStats()
    pipe = SortingPipeline(robot, fake_mapper(), world.observe, stats=stats)

    original = pipe._run_cycle
    pipe._run_cycle = lambda b, o: (original(b, o) and (world.remove_ball(b) or True))

    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    t.join(5.0)

    assert not t.is_alive()
    assert stats.unreachable >= 1, "영역 밖 공이 통계에 안 잡혔다"
    assert stats.total >= 1, "닿는 공은 처리했어야 한다"


# ── 딜레이 보호장치가 실제로 연결됐는가 ────────────────────────────────────
def test_watchdog_stop_reaches_robot_mid_motion():
    """★ 워치독 정지가 `robot.should_abort` 통로로 이어지는가.

    이 통로가 끊겨 있으면 지연이 아무리 커도 이미 시작한 동작은 끝까지 간다.
    """
    world = FakeWorld(balls=[("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = FakeRobot()
    wd = LatencyWatchdog(TH)
    SortingPipeline(robot, fake_mapper(), world.observe, watchdog=wd)

    assert robot.should_abort is not None, "파이프라인이 훅을 안 걸었다"
    assert robot.should_abort() is False

    wd.report(CAMERA, 2000)
    assert wd.level is Level.STOP
    assert robot.should_abort() is True, "워치독 정지가 로봇까지 안 닿는다"


def test_hold_level_blocks_new_motion_but_not_current():
    """보류는 새 동작만 막는다 — 하던 동작은 마치게 둔다."""
    wd = LatencyWatchdog(TH)
    wd.report(CAMERA, 1000)                     # HOLD

    assert wd.level is Level.HOLD
    assert not wd.may_start_motion(), "보류인데 새 동작을 허용한다"
    assert not wd.should_abort(), "보류가 진행 중인 동작까지 끊고 있다"


def test_pipeline_waits_while_held_then_resumes():
    """지연이 보류 단계면 멈춰 있다가, 풀리면 다시 진행한다."""
    world = FakeWorld(balls=[("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = FakeRobot()
    wd = LatencyWatchdog(TH)
    pipe = SortingPipeline(robot, fake_mapper(), world.observe, watchdog=wd)

    original = pipe._run_cycle
    pipe._run_cycle = lambda b, o: (original(b, o) and (world.remove_ball(b) or True))

    wd.report(CAMERA, 1000)                     # 시작 전부터 보류 상태
    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    threading.Event().wait(0.4)

    assert pipe.state is State.PAUSED
    assert "place" not in robot.names_called(), "보류 중에 동작을 시작했다"

    for _ in range(TH.recover_samples * 4):     # 지연 회복
        wd.report(CAMERA, 10)
    t.join(5.0)

    assert not t.is_alive()
    assert pipe.state is State.DONE
    assert "place" in robot.names_called(), "회복했는데 진행하지 않았다"


def test_injected_delay_takes_the_same_path_as_real_delay():
    """데모의 지연 주입이 진짜 지연과 같은 경로를 타는가.

    다른 경로를 타면 데모가 '보여주기용 연출'이 되고, 실제 보호장치가
    동작한다는 증거가 되지 못한다.
    """
    robot = FakeRobot()
    world = FakeWorld(balls=[], bins={})
    wd = LatencyWatchdog(TH)
    SortingPipeline(robot, fake_mapper(), world.observe, watchdog=wd)

    wd.report(CAMERA, 10)
    assert robot.should_abort() is False

    wd.inject(2000)                              # 슬라이더를 올린 것과 같다
    wd.report(CAMERA, 10)
    assert robot.should_abort() is True, "주입한 지연이 실제 정지 경로를 안 탄다"


def test_watchdog_absent_is_harmless():
    """워치독 없이도 파이프라인이 그대로 돈다(테스트·시뮬 편의)."""
    world = FakeWorld(balls=[("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = FakeRobot()
    pipe = SortingPipeline(robot, fake_mapper(), world.observe, watchdog=None)

    original = pipe._run_cycle
    pipe._run_cycle = lambda b, o: (original(b, o) and (world.remove_ball(b) or True))
    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    t.join(5.0)
    assert not t.is_alive() and pipe.state is State.DONE
