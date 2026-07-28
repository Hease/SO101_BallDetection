# -*- coding: utf-8 -*-
"""하드웨어를 갈아끼워도 위쪽 코드가 그대로 도는가.

이 파일이 지키려는 주장은 하나다.

    **로봇이나 카메라를 print 로 바꿔도 상태머신·안전장치·비전이 그대로 돈다.**

주장이 맞다면 경계가 진짜인 것이고, 새 하드웨어가 들어와도 어댑터 하나만
쓰면 된다. 틀리다면 어딘가에서 위쪽 코드가 하드웨어를 직접 들여다보고 있다는 뜻이다.
"""
from __future__ import annotations

import threading

import numpy as np
import pytest

from sorting import drivers
from sorting.drivers import PrintCamera, PrintRobot, make_camera, make_robot
from sorting.pipeline import Observation, SortingPipeline, State
from sorting.ports import CameraPort, RobotPort, missing_methods
from sorting.stats import SessionStats
from sorting.vision import observe
from sorting.watchdog import CAMERA, LatencyWatchdog, Thresholds

from .fakes import FakeRobot, fake_mapper


# ── 계약 검사 ──────────────────────────────────────────────────────────────
@pytest.mark.parametrize("name", sorted(drivers.ROBOTS))
def test_every_registered_robot_satisfies_the_contract(name):
    """등록된 로봇 드라이버는 전부 RobotPort 를 만족해야 한다.

    새 로봇을 붙인 사람이 여기서 바로 무엇을 빠뜨렸는지 알 수 있다.
    """
    try:
        robot = drivers.ROBOTS[name](verbose=False)
    except Exception as exc:
        pytest.skip(f"{name} 생성 불가(하드웨어 없음): {exc}")

    gaps = missing_methods(robot, RobotPort)
    assert not gaps, f"'{name}' 드라이버에 빠진 것: {gaps}"
    assert isinstance(robot, RobotPort)
    robot.close()


@pytest.mark.parametrize("name", sorted(drivers.CAMERAS))
def test_every_registered_camera_satisfies_the_contract(name):
    try:
        cam = drivers.CAMERAS[name](verbose=False)
    except Exception as exc:
        pytest.skip(f"{name} 생성 불가(하드웨어 없음): {exc}")

    assert not missing_methods(cam, CameraPort)
    assert isinstance(cam, CameraPort)


def test_unknown_driver_names_the_alternatives():
    """이름을 틀렸을 때 무엇을 쓸 수 있는지 알려줘야 한다."""
    with pytest.raises(SystemExit) as exc:
        make_robot("없는로봇")
    assert "so101" in str(exc.value) and "print" in str(exc.value)

    with pytest.raises(SystemExit) as exc:
        make_camera("없는카메라")
    assert "replay" in str(exc.value)


# ── print 로봇이 '거짓말하지 않는가' ───────────────────────────────────────
def test_print_robot_reports_what_it_would_do():
    robot = PrintRobot(verbose=False)
    robot.home()
    robot.pick((0.20, 0.05))
    robot.place((0.15, -0.10))

    joined = " ".join(robot.log)
    assert "홈 자세" in joined and "집기" in joined and "놓기" in joined
    assert "그리퍼" in joined, "그리퍼 동작도 보고해야 한다"


def test_print_robot_estop_actually_blocks():
    """가짜라도 E-STOP 은 진짜로 동작을 막아야 한다.

    안 그러면 print 모드에서 안전장치를 시연할 수 없고, 위쪽 코드가
    진짜 로봇과 다른 경로를 타게 된다.
    """
    robot = PrintRobot(verbose=False)
    robot.estop()
    assert robot.estopped

    before = len(robot.log)
    robot.move_to((0.2, 0.0, 0.1))
    robot.pick((0.2, 0.0))
    assert len(robot.log) == before, "정지 상태인데 명령이 실행됐다"

    robot.release_estop()
    robot.move_to((0.2, 0.0, 0.1))
    assert len(robot.log) > before


def test_print_robot_grip_drives_holding_judgement():
    """파지 성공 판정이 진짜 로봇과 같은 규칙을 따르는가."""
    robot = PrintRobot(verbose=False)
    robot.set_grip(100)
    assert robot.holding_object() is True
    robot.set_grip(0)
    assert robot.holding_object() is False


def test_print_robot_pose_changes_so_twin_moves():
    """3D 트윈이 멈춰 보이지 않도록 자세가 실제로 변해야 한다."""
    robot = PrintRobot(verbose=False)
    first = list(robot.joint_angles_deg())
    robot.move_to((0.20, 0.15, 0.10))
    assert robot.joint_angles_deg() != first


def test_print_robot_enforces_measured_workspace(isolate_calibration):
    """print 모드에서도 도달 한계가 살아 있어야 데모가 성립한다."""
    from sorting.robot import OutOfReach
    from sorting.workspace import Workspace

    Workspace.from_trace([(0.12, -0.12, 0.0), (0.26, -0.12, 0.0),
                          (0.26, 0.12, 0.2), (0.12, 0.12, 0.0)]).save()
    robot = PrintRobot(verbose=False)

    robot.move_to((0.20, 0.0, 0.1))            # 안쪽 — 통과
    with pytest.raises(OutOfReach):
        robot.move_to((0.44, 0.0, 0.1))        # IK 는 푸는 좌표, 영역이 막는다


# ── print 카메라가 실제로 검출 가능한 장면을 주는가 ────────────────────────
def test_print_camera_frames_are_detectable():
    """합성 화면이 진짜 검출기를 통과해야 의미가 있다.

    아무 그림이나 내보내면 비전 계층이 빈 장면만 보게 되어, 파이프라인이
    도는 것처럼 보여도 실제로는 아무것도 검증하지 못한다.
    """
    with PrintCamera(verbose=False) as cam:
        rgb, depth, fid = cam.read_blocking(0)

    assert rgb is not None and rgb.dtype == np.uint8
    assert depth is None, "깊이가 없어도 나머지는 돌아야 한다"
    assert fid == 1

    scene = observe(rgb)
    assert len(scene.balls_of("red")) == 1, "빨간 공이 검출돼야 한다"
    assert len(scene.balls_of("blue")) == 1
    assert set(scene.regions) == {"red", "blue"}, "구역도 검출돼야 한다"


def test_print_camera_frame_id_advances():
    with PrintCamera(verbose=False) as cam:
        _r, _d, a = cam.read_blocking(0)
        _r, _d, b = cam.read_blocking(a)
    assert b > a


def test_print_camera_can_remove_picked_object():
    cam = PrintCamera(verbose=False)
    before = len(cam.balls)
    assert cam.remove_at(250, 300) is True
    assert len(cam.balls) == before - 1
    assert cam.remove_at(9999, 9999) is False


# ── 통째로 갈아끼워도 도는가 (이 파일의 핵심) ──────────────────────────────
def test_whole_pipeline_runs_with_print_robot():
    """★ 로봇을 print 로 바꿔도 상태머신이 끝까지 돈다."""
    from .fakes import FakeWorld

    world = FakeWorld(balls=[("red", 300, 240), ("blue", 360, 260)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = PrintRobot(verbose=False)
    stats = SessionStats()
    pipe = SortingPipeline(robot, fake_mapper(), world.observe, stats=stats)

    original = pipe._run_cycle
    pipe._run_cycle = lambda b, o: (original(b, o) and (world.remove_ball(b) or True))

    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    t.join(10.0)

    assert not t.is_alive()
    assert pipe.state is State.DONE
    assert stats.total == 2, "print 로봇으로도 두 개를 처리했어야 한다"
    assert any("놓기" in line for line in robot.log)


def test_safety_still_works_with_print_robot():
    """★ 하드웨어를 갈아끼워도 안전장치 배선이 유지되는가."""
    from .fakes import FakeWorld

    world = FakeWorld(balls=[("red", 300, 240)],
                      bins={"red": (100, 100), "blue": (540, 100)})
    robot = PrintRobot(verbose=False)
    wd = LatencyWatchdog(Thresholds(warn_ms=300, hold_ms=800, stop_ms=1500,
                                    recover_samples=2))
    SortingPipeline(robot, fake_mapper(), world.observe, watchdog=wd)

    assert robot.should_abort is not None, "파이프라인이 훅을 못 걸었다"
    assert robot.should_abort() is False

    wd.report(CAMERA, 2000)
    assert robot.should_abort() is True, "워치독 정지가 print 로봇까지 안 닿는다"


def test_vision_runs_on_print_camera_frames():
    """★ 카메라를 print 로 바꿔도 비전 계층이 그대로 돈다."""
    with make_camera("print", verbose=False) as cam:
        rgb, _depth, _fid = cam.read_blocking(0)

    scene = observe(rgb)
    assert scene.balls, "합성 화면에서 공을 못 찾았다"
    assert scene.regions, "합성 화면에서 구역을 못 찾았다"


def test_swapping_robot_does_not_touch_upper_layers():
    """같은 파이프라인 코드가 서로 다른 로봇 두 개로 모두 돌아간다."""
    from .fakes import FakeWorld

    results = {}
    for label, robot in (("print", PrintRobot(verbose=False)),
                         ("fake", FakeRobot())):
        world = FakeWorld(balls=[("red", 300, 240)],
                          bins={"red": (100, 100), "blue": (540, 100)})
        stats = SessionStats()
        pipe = SortingPipeline(robot, fake_mapper(), world.observe, stats=stats)
        original = pipe._run_cycle
        pipe._run_cycle = lambda b, o, _o=original: (
            _o(b, o) and (world.remove_ball(b) or True))

        t = threading.Thread(target=pipe.run, daemon=True)
        t.start()
        t.join(10.0)
        results[label] = (pipe.state, stats.total)

    assert results["print"] == results["fake"], \
        f"드라이버에 따라 결과가 달라졌다: {results}"


def test_fast_robot_does_not_resort_the_same_ball():
    """★ 하드웨어가 빨라져도 같은 공을 다시 집지 않는가 (회귀).

    트래커는 공이 사라져도 forget_frames 동안 계속 보고한다. 놓자마자 재스캔하면
    같은 공이 후보로 남아 있어 다시 집게 된다. 지금까지는 "이동에 몇 초 걸리니
    그 사이 트래커가 잊는다"에 가려져 있었는데, 그건 로봇이 트래커보다 느리다는
    가정이다. print 드라이버(즉시 동작)로 갈아끼우자 공 2개가 42,485개로 세어졌다.

    비전 쪽은 그대로 두고 **로봇만 빠른 것으로 바꿔도** 결과가 같아야 한다.
    """
    from sorting.tracking import BallTracker

    cam = PrintCamera(verbose=False)
    tracker = BallTracker()
    robot = PrintRobot(verbose=False)
    stats = SessionStats()

    state = {"obs": None}

    def look():
        rgb, _d, _f = cam.read_blocking(0)
        scene = observe(rgb)
        state["obs"] = Observation(scene=scene, balls=tracker.update(scene))
        return state["obs"]

    for _ in range(4):        # 트래커가 공을 '확정'할 때까지 몇 프레임
        look()

    pipe = SortingPipeline(robot, fake_mapper(), look, stats=stats)
    original = pipe._run_cycle

    def cycle(ball, obs):
        ok = original(ball, obs)
        if ok:
            cam.remove_at(ball.u, ball.v)     # 로봇이 집어갔으니 장면에서 사라진다
        return ok

    pipe._run_cycle = cycle

    t = threading.Thread(target=pipe.run, daemon=True)
    t.start()
    t.join(20.0)

    assert not t.is_alive(), "끝나지 않았다 — 같은 공을 무한히 다시 집는 중"
    assert stats.total == 2, f"공 2개인데 {stats.total}개로 셌다"
    assert stats.per_color == {"red": 1, "blue": 1}
