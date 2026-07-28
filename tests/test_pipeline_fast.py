# -*- coding: utf-8 -*-
"""파이프라인의 *판단*을 물리 없이 빠르게 검증한다.

test_pipeline.py 는 헤드리스 MuJoCo 로 실제 IK·물리를 거치므로 87초가 걸린다.
그건 "이 좌표가 진짜 팔 범위 안인가" 같은 물리적 제약까지 확인하는 값진 테스트라
남겨두되, **순서·분기·재시도 같은 판단**은 여기서 밀리초 안에 확인한다.

요구사항이 갑자기 바뀌어 급히 고쳐야 할 때 5초 피드백과 100초 피드백은 완전히
다른 작업이 된다. 이 파일이 그 5초를 만든다.
"""
from __future__ import annotations

import threading

import pytest

from sorting import config as cfg
from sorting.pipeline import SortingPipeline, State
from sorting.stats import SessionStats

from .fakes import FakeRobot, FakeWorld, fake_mapper


def run_until_done(pipe: SortingPipeline, world: FakeWorld, timeout: float = 5.0):
    """놓을 때마다 공을 치우고, 끝날 때까지 돌린다."""
    original = pipe._run_cycle

    def cycle(ball, obs):
        ok = original(ball, obs)
        if ok:
            world.remove_ball(ball)      # 목록 0번이 아니라 '처리한 그 공'
        return ok

    pipe._run_cycle = cycle
    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    t.join(timeout)
    assert not t.is_alive(), "파이프라인이 시간 안에 끝나지 않았다"


def test_sorts_every_ball_then_finishes():
    world = FakeWorld(balls=[("red", 300, 240), ("blue", 360, 260)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = FakeRobot()
    stats = SessionStats()
    events = []
    pipe = SortingPipeline(robot, fake_mapper(), world.observe, stats=stats,
                           on_event=lambda n, d: events.append((n, d)))

    run_until_done(pipe, world)

    assert pipe.state is State.DONE
    assert stats.per_color == {"red": 1, "blue": 1}
    assert any(n == "complete" for n, _ in events)
    assert robot.names_called().count("place") == 2


def test_place_follows_moved_bin():
    """★ 과제 핵심: 사이클 도중 bin 을 옮기면 다음 공은 새 자리로 간다."""
    # 두 공은 실제 지름(40mm)보다 넓게 떨어뜨린다 — 8mm 간격은 물리적으로 불가능하고,
    # 그러면 "방금 처리한 자리" 쿨다운이 둘을 같은 공으로 본다.
    world = FakeWorld(balls=[("red", 250, 240), ("red", 420, 260)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = FakeRobot()
    mapper = fake_mapper()
    pipe = SortingPipeline(robot, mapper, world.observe)

    original_place = robot.place

    def spy(bin_xy):
        original_place(bin_xy)
        if robot.names_called().count("place") == 1:
            world.move_bin("red", 120, 380)      # 첫 공을 놓은 직후 옮긴다

    robot.place = spy
    run_until_done(pipe, world)

    placed = [c[1] for c in robot.calls if c[0] == "place"]
    assert len(placed) == 2
    assert placed[0] == pytest.approx(mapper.to_robot(100, 100), abs=1e-6)
    assert placed[1] == pytest.approx(mapper.to_robot(120, 380), abs=1e-6), \
        "bin 을 옮겼는데 옛 좌표에 놓았다 — 좌표를 어딘가 캐싱하고 있다"


def test_grasp_failure_retries_then_gives_up():
    world = FakeWorld(balls=[("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = FakeRobot(holding=False)             # 항상 파지 실패
    stats = SessionStats()
    pipe = SortingPipeline(robot, fake_mapper(), world.observe, stats=stats)

    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    t.join(5.0)

    attempts = [c[2] for c in robot.calls if c[0] == "pick"]
    assert attempts == list(range(cfg.ROBOT.max_retries + 1)), \
        f"재시도 횟수가 설정과 다르다: {attempts}"
    assert sum(stats.failures.values()) == 1
    assert pipe.state is State.DONE, "포기한 공을 계속 다시 고르면 안 된다"


def test_unreachable_ball_is_skipped_not_fatal():
    """팔 범위 밖의 공 하나 때문에 사이클 전체가 죽으면 안 된다."""
    world = FakeWorld(balls=[("red", 620, 460), ("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = FakeRobot(reach_limit=0.28)
    stats = SessionStats()
    pipe = SortingPipeline(robot, fake_mapper(), world.observe, stats=stats)

    run_until_done(pipe, world)

    assert pipe.state is State.DONE
    assert stats.unreachable >= 1
    assert stats.total >= 1, "닿는 공은 처리했어야 한다"


def test_missing_bin_waits_instead_of_guessing():
    """bin 이 안 보이면 아무 데나 놓지 않고 기다린다."""
    world = FakeWorld(balls=[("red", 300, 240)], bins={"blue": (540, 100)})
    robot = FakeRobot()
    pipe = SortingPipeline(robot, fake_mapper(), world.observe)

    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    threading.Event().wait(1.0)
    assert pipe.state is State.SCAN
    assert "place" not in robot.names_called()
    pipe.request_stop()
    t.join(5.0)
    assert not t.is_alive()


def test_intrusion_pauses_and_resumes_from_scan():
    world = FakeWorld(balls=[("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = FakeRobot()
    states = []
    pipe = SortingPipeline(robot, fake_mapper(), world.observe,
                           on_event=lambda n, d: states.append(d.get("state"))
                           if n == "state" else None)
    original = pipe._run_cycle
    pipe._run_cycle = lambda b, o: (original(b, o) and (world.remove_ball(b) or True))

    world.intruded = True
    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    threading.Event().wait(0.6)

    assert pipe.state is State.PAUSED
    assert robot.estopped, "일시정지 중에는 E-STOP 이 걸려 있어야 한다"

    world.intruded = False
    t.join(5.0)
    assert not t.is_alive()
    assert pipe.state is State.DONE
    assert State.PAUSED.value in states


def test_pick_happens_before_place():
    """순서가 뒤집히지 않는가 — 집기 전에 놓으러 가면 안 된다."""
    world = FakeWorld(balls=[("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = FakeRobot()
    pipe = SortingPipeline(robot, fake_mapper(), world.observe)
    run_until_done(pipe, world)

    names = robot.names_called()
    assert names.index("pick") < names.index("place")
    assert names[0] == "home", "시작할 때 안전 자세를 먼저 잡아야 한다"
