# -*- coding: utf-8 -*-
"""상태머신을 카메라·로봇 없이 돌려본다.

가짜 관측(FakeWorld)을 물려서, 로봇은 헤드리스 MuJoCo 로 실제 IK/물리를 거치게
한다. 그래서 "좌표가 팔 범위 안인가" 같은 진짜 제약도 같이 검증된다.

핵심 검증 두 가지:
  · bin 을 사이클 도중에 옮기면 다음 공이 새 자리로 가는가 (과제 핵심 요구사항)
  · 손이 감지되면 멈추고, 빠지면 다시 시작하는가
"""
from __future__ import annotations

import os
import threading

import numpy as np
import pytest

os.environ.setdefault("SOARM_HEADLESS", "1")

from sorting import config as cfg
from sorting.mapping import Mapper
from sorting.pipeline import Observation, SortingPipeline, State
from sorting.robot import RobotController
from sorting.stats import SessionStats
from sorting.vision import Ball, Region, Scene


# 픽셀 → 로봇좌표를 단순 선형으로 잇는 가짜 캘리브레이션.
# 640x480 화면이 로봇 앞 x=0.15~0.30, y=-0.12~+0.12 에 대응한다고 본다.
def _fake_mapper() -> Mapper:
    src = np.float32([[0, 0], [640, 0], [640, 480], [0, 480]])
    dst = np.float32([[0.15, 0.12], [0.15, -0.12], [0.30, -0.12], [0.30, 0.12]])
    import cv2
    return Mapper(cv2.getPerspectiveTransform(src, dst))


class FakeWorld:
    """공과 bin 이 있는 가짜 책상. 로봇이 놓으면 공이 사라진다."""

    def __init__(self, balls, bins, pick_zone=True):
        self.balls = list(balls)                 # [(color, u, v)]
        self.bins = dict(bins)                   # {color: (u, v)}
        self.pick_zone = pick_zone
        self.intruded = False
        self.observations = 0

    def move_bin(self, color, u, v):
        self.bins[color] = (u, v)

    def observe(self) -> Observation:
        self.observations += 1
        regions = {
            c: Region(c, u, v, (u - 60, v - 60, 120, 120), 14400.0)
            for c, (u, v) in self.bins.items()
        }
        zone = Region("orange", 320, 240, (0, 0, 640, 480), 307200.0) \
            if self.pick_zone else None
        balls = [Ball(c, u, v, 20, 1200.0) for c, u, v in self.balls]
        scene = Scene(balls=balls, regions=regions, pick_zone=zone)
        return Observation(scene=scene, balls=balls, intruded=self.intruded)

    def consume_nearest(self):
        """로봇이 공 하나를 집어간 것으로 친다."""
        if self.balls:
            self.balls.pop(0)


# 이 파일은 전부 느린 테스트다 — 헤드리스 MuJoCo 로 실제 IK·물리를 거친다.
# 빠른 피드백이 필요할 땐  pytest -m "not slow"
pytestmark = pytest.mark.slow

@pytest.fixture
def robot():
    r = RobotController(real=False)
    yield r
    r.close()


def _run(pipeline, timeout=120):
    t = threading.Thread(target=pipeline.run, daemon=True)
    t.start()
    t.join(timeout)
    assert not t.is_alive(), "파이프라인이 시간 안에 끝나지 않았다"


def test_sorts_until_empty(robot):
    """공이 없어질 때까지 돌고 CEREMONY 로 끝난다."""
    world = FakeWorld(balls=[("red", 300, 240), ("blue", 360, 260)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    stats = SessionStats()
    events = []

    # 놓기가 끝날 때마다 그 공을 책상에서 치운다
    pipe = SortingPipeline(robot, _fake_mapper(), world.observe, stats=stats,
                           on_event=lambda n, d: events.append((n, d)))
    original = pipe._run_cycle

    def cycle(ball, obs):
        ok = original(ball, obs)
        if ok:
            world.consume_nearest()
        return ok

    pipe._run_cycle = cycle
    _run(pipe)

    assert pipe.state is State.DONE
    assert stats.total == 2, f"2개를 분류해야 하는데 {stats.total}개"
    assert stats.per_color == {"red": 1, "blue": 1}
    assert any(n == "complete" for n, _ in events)


def test_place_follows_moved_bin(robot):
    """★ 과제 핵심: 사이클 도중 bin 을 옮기면 다음 공은 새 자리로 간다."""
    world = FakeWorld(balls=[("red", 300, 240), ("red", 320, 250)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    mapper = _fake_mapper()
    placed_at = []

    pipe = SortingPipeline(robot, mapper, world.observe)
    original_place = robot.place

    def spy_place(bin_xy):
        placed_at.append(tuple(bin_xy))
        original_place(bin_xy)
        world.consume_nearest()
        if len(placed_at) == 1:                  # 첫 공을 놓은 직후 bin 을 옮긴다
            world.move_bin("red", 120, 380)

    robot.place = spy_place
    _run(pipe)

    assert len(placed_at) == 2, f"두 번 놓아야 하는데 {len(placed_at)}번"
    expected_first = mapper.to_robot(100, 100)
    expected_second = mapper.to_robot(120, 380)
    assert placed_at[0] == pytest.approx(expected_first, abs=1e-6)
    assert placed_at[1] == pytest.approx(expected_second, abs=1e-6), \
        "bin 을 옮겼는데 옛 좌표에 놓았다 — 좌표를 어딘가 캐싱하고 있다"
    assert placed_at[0] != placed_at[1]


def test_intrusion_pauses_and_resumes(robot):
    """손이 보이면 멈추고, 빠지면 재개한다."""
    world = FakeWorld(balls=[("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    states = []
    pipe = SortingPipeline(robot, _fake_mapper(), world.observe,
                           on_event=lambda n, d: states.append(d.get("state"))
                           if n == "state" else None)

    original = pipe._run_cycle

    def cycle(ball, obs):
        ok = original(ball, obs)
        if ok:
            world.consume_nearest()
        return ok

    pipe._run_cycle = cycle

    world.intruded = True                        # 시작하자마자 손이 들어와 있다
    thread = threading.Thread(target=pipe.run, daemon=True)
    thread.start()

    deadline = threading.Event()
    deadline.wait(2.0)
    assert pipe.state is State.PAUSED, f"멈춰야 하는데 {pipe.state}"
    assert robot.estopped, "일시정지 중에는 E-STOP 이 걸려 있어야 한다"

    world.intruded = False                       # 손을 뺐다
    thread.join(120)
    assert not thread.is_alive()
    assert pipe.state is State.DONE
    assert State.PAUSED.value in states and states[-1] == State.DONE.value


def test_missing_bin_does_not_crash(robot):
    """bin 이 안 보이면 집지 않고 기다린다(공을 들고 헤매지 않는다)."""
    world = FakeWorld(balls=[("red", 300, 240)], bins={"blue": (540, 100)})
    pipe = SortingPipeline(robot, _fake_mapper(), world.observe)

    thread = threading.Thread(target=pipe.run, daemon=True)
    thread.start()
    threading.Event().wait(2.0)
    assert pipe.state is State.SCAN
    pipe.request_stop()
    thread.join(30)
    assert not thread.is_alive()


def test_grasp_failure_is_retried(robot, monkeypatch):
    """잡힌 게 없으면 재시도하고, 계속 실패하면 그 공을 건너뛴다."""
    world = FakeWorld(balls=[("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    stats = SessionStats()
    monkeypatch.setattr(robot, "holding_object", lambda: False)   # 항상 파지 실패

    attempts = []
    original_pick = robot.pick
    monkeypatch.setattr(robot, "pick",
                        lambda xy, attempt=0: (attempts.append(attempt),
                                               original_pick(xy, attempt))[1])

    pipe = SortingPipeline(robot, _fake_mapper(), world.observe, stats=stats)
    thread = threading.Thread(target=pipe.run, daemon=True)
    thread.start()
    threading.Event().wait(60)
    pipe.request_stop()
    thread.join(30)

    assert attempts == list(range(cfg.ROBOT.max_retries + 1)), \
        f"재시도 횟수가 설정과 다르다: {attempts}"
    assert stats.retries >= cfg.ROBOT.max_retries
    assert sum(stats.failures.values()) >= 1
