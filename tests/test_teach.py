# -*- coding: utf-8 -*-
"""티칭 도구 검증 — 하드웨어 없이.

`TeachSession` 이 reader 를 주입받게 만든 덕분에 서보 없이 전부 검증된다.
여기서 지키려는 것 두 가지:
  · 손으로 훑은 궤적이 실제 작업영역이 되는가
  · **서보가 응답하지 않을 때 막히지 않고 수동으로 넘어가는가**
"""
from __future__ import annotations

import math

import pytest

from sorting.teach import Pose, PoseLibrary, ReadFailed, TeachSession
from sorting.workspace import Workspace


def circle_trace(r=0.25, z=0.05, n=24):
    """반경 r 의 원을 도는 가짜 손 궤적."""
    return [(r * math.cos(2 * math.pi * i / n),
             r * math.sin(2 * math.pi * i / n), z) for i in range(n)]


class ScriptedReader:
    """미리 정해둔 좌표를 순서대로 돌려주는 가짜 서보 읽기."""

    def __init__(self, points, fail_after=None):
        self.points = list(points)
        self.fail_after = fail_after
        self.reads = 0

    def __call__(self):
        self.reads += 1
        if self.fail_after is not None and self.reads > self.fail_after:
            raise ReadFailed("가짜 서보 무응답")
        xyz = self.points[(self.reads - 1) % len(self.points)]
        return xyz, (0.0, 30.0, -45.0, 0.0, 0.0)


# ── 경계 훑기 ──────────────────────────────────────────────────────────────
def test_trace_becomes_workspace():
    """훑은 궤적이 그대로 작업영역이 되는가."""
    trace = circle_trace(r=0.25)
    session = TeachSession(reader=ScriptedReader(trace))

    points = session.trace_boundary(seconds=0.3, hz=200)
    assert len(points) > 3

    ws = Workspace.from_trace(points)
    assert ws.contains(0.0, 0.0, 0.05), "원 안쪽은 통과해야 한다"
    assert not ws.contains(0.40, 0.0, 0.05), "원 밖은 거절해야 한다"


def test_trace_skips_occasional_read_misses():
    """중간에 한두 번 못 읽었다고 측정 전체를 버리지 않는다."""
    trace = circle_trace()

    class Flaky(ScriptedReader):
        def __call__(self):
            self.reads += 1
            if self.reads % 4 == 0:              # 4번에 한 번 실패
                raise ReadFailed("일시적 무응답")
            return self.points[self.reads % len(self.points)], ()

    session = TeachSession(reader=Flaky(trace))
    points = session.trace_boundary(seconds=0.3, hz=200)
    assert len(points) > 3, "간헐적 실패에 측정이 통째로 죽었다"


def test_trace_gives_up_when_servo_is_really_dead():
    """계속 못 읽으면 조용히 빈 결과를 주지 말고 실패를 알려야 한다.

    빈 결과를 돌려주면 '측정했는데 영역이 이상하다'로 보여 원인을 찾기 어렵다.
    """
    session = TeachSession(reader=ScriptedReader(circle_trace(), fail_after=0))
    with pytest.raises(ReadFailed):
        session.trace_boundary(seconds=5.0, hz=200, give_up_after_s=0.1)


def test_workspace_from_trace_rejects_ik_reachable_far_point():
    """★ 실측 영역이 IK 가 통과시키던 0.44m 를 막는가."""
    ws = Workspace.from_trace(circle_trace(r=0.26))
    verdict = ws.check(0.44, 0.0, 0.05)
    assert not verdict.ok and verdict.margin_m < 0


# ── 포즈 라이브러리 ────────────────────────────────────────────────────────
def test_pose_roundtrip():
    lib = PoseLibrary()
    lib.add(Pose("bin_red", (0.18, -0.12, 0.06), (1.0, 2.0, 3.0, 4.0, 5.0)))

    again = PoseLibrary()
    pose = again.get("bin_red")
    assert pose is not None
    assert pose.xyz == pytest.approx((0.18, -0.12, 0.06))
    assert pose.joints_deg == pytest.approx((1.0, 2.0, 3.0, 4.0, 5.0))


def test_pose_remove():
    lib = PoseLibrary()
    lib.add(Pose("temp", (0.2, 0.0, 0.1)))
    assert lib.remove("temp") is True
    assert lib.remove("temp") is False
    assert PoseLibrary().get("temp") is None


def test_pose_names_sorted():
    lib = PoseLibrary()
    for n in ("bin_red", "aaa", "zzz"):
        lib.add(Pose(n, (0.2, 0.0, 0.1)))
    assert lib.names() == ["aaa", "bin_red", "zzz"]


def test_manual_pose_is_marked_manual(isolate_calibration):
    """서보를 못 읽어 손으로 넣은 좌표는 '수동'으로 남아야 한다."""
    lib = PoseLibrary()
    lib.add(Pose("bin_blue", (0.2, 0.1, 0.05)), manual=True,
            note="서보2 무응답 — 자로 측정")

    item = isolate_calibration.get("poses", None)
    assert item.source == "manual"
    assert "자로 측정" in item.note


# ── 저장소 연동 ────────────────────────────────────────────────────────────
def test_measured_workspace_is_marked_measured(isolate_calibration):
    Workspace.from_trace(circle_trace()).save(note="손으로 훑음 (24점)")
    item = isolate_calibration.get("workspace", None)
    assert item.source == "measured"
    assert "훑음" in item.note


def test_teaching_removes_item_from_missing_list(isolate_calibration):
    """티칭하면 '아직 안 잰 값' 목록에서 빠져야 한다."""
    assert "workspace" in {k for k, _d, _h in isolate_calibration.missing()}
    Workspace.from_trace(circle_trace()).save()
    assert "workspace" not in {k for k, _d, _h in isolate_calibration.missing()}


def test_heights_can_be_set_manually_when_read_fails(isolate_calibration):
    """자동 읽기가 실패해도 수동 경로로 값이 들어가면 진행할 수 있다."""
    isolate_calibration.set_manual("robot.z_grasp", 0.006, "서보4 무응답, 자로 측정")

    item = isolate_calibration.get("robot.z_grasp", 0.005)
    assert item.value == 0.006 and item.source == "manual"
    assert "robot.z_grasp" not in {k for k, _d, _h in isolate_calibration.missing()}
