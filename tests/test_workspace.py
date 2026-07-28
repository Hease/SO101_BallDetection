# -*- coding: utf-8 -*-
"""작업영역(도달 한계) 검증.

이 파일이 지키는 가장 중요한 것: **IK 가 통과시키는 좌표를 실측 영역이 막는가.**
IK 를 직접 샘플링해 보면 반경 0.44m 까지 잔차 0mm 로 풀린다. 즉 IK 잔차만 믿으면
한계가 사실상 없다. 손으로 훑어 잰 영역이 그 구멍을 메운다.
"""
from __future__ import annotations

import pytest

from sorting.workspace import Workspace

# 손으로 훑었다고 가정한 궤적 (로봇좌표, m)
TRACE = [
    (0.13, -0.14, 0.02), (0.20, -0.17, 0.03), (0.28, -0.12, 0.05),
    (0.30, 0.00, 0.20), (0.28, 0.12, 0.05), (0.20, 0.17, 0.03),
    (0.13, 0.14, 0.02), (0.11, 0.00, 0.01),
]


@pytest.fixture
def ws():
    return Workspace.from_trace(TRACE)


def test_inside_is_allowed(ws):
    assert ws.check(0.20, 0.00, 0.10).ok
    assert ws.contains(0.22, 0.05, 0.08)


def test_ik_reachable_but_unsafe_is_rejected(ws):
    """★ 핵심: IK 가 잔차 0mm 로 푸는 0.44m 를 영역이 거절해야 한다."""
    verdict = ws.check(0.44, 0.0, 0.10)
    assert not verdict.ok
    assert "밖" in verdict.reason
    assert verdict.margin_m < 0


def test_rejection_says_how_far_out(ws):
    """거절할 때 얼마나 벗어났는지 알려줘야 조작자가 판단할 수 있다."""
    verdict = ws.check(0.35, 0.0, 0.10)
    assert not verdict.ok
    assert "mm" in verdict.reason
    assert verdict.margin_m < 0


def test_height_limits(ws):
    too_high = ws.check(0.20, 0.0, 0.50)
    too_low = ws.check(0.20, 0.0, -0.05)
    assert not too_high.ok and "높음" in too_high.reason
    assert not too_low.ok and "낮음" in too_low.reason


def test_z_range_comes_from_the_trace(ws):
    """z 범위도 훑은 궤적에서 나온다 — 손으로 정한 숫자가 아니다."""
    assert ws.z_min == pytest.approx(min(p[2] for p in TRACE))
    assert ws.z_max == pytest.approx(max(p[2] for p in TRACE))


def test_margin_is_positive_inside_negative_outside(ws):
    assert ws.margin_m(0.20, 0.0) > 0
    assert ws.margin_m(0.60, 0.0) < 0


def test_verdict_is_truthy(ws):
    assert bool(ws.check(0.20, 0.0, 0.10)) is True
    assert bool(ws.check(0.90, 0.0, 0.10)) is False


def test_convex_hull_ignores_hand_jitter():
    """손으로 그린 궤적은 떨리고 되짚는다 — 껍질을 취해 단순 도형으로 만든다."""
    jittery = TRACE + [(0.20, 0.0, 0.05), (0.19, 0.01, 0.05),   # 안쪽 되짚기
                       (0.21, -0.01, 0.05)]
    ws = Workspace.from_trace(jittery)
    assert len(ws.polygon_xy) <= len(TRACE), "안쪽 점이 경계에 남았다"
    assert ws.contains(0.20, 0.0, 0.05)


def test_shrink_adds_safety_margin():
    """여유를 두고 싶을 때 경계를 안쪽으로 줄일 수 있다."""
    full = Workspace.from_trace(TRACE)
    tight = Workspace.from_trace(TRACE, shrink_m=0.03)
    edge = (0.295, 0.0)
    assert full.margin_m(*edge) > tight.margin_m(*edge)


def test_too_few_points_raises():
    with pytest.raises(ValueError):
        Workspace.from_trace([(0.1, 0.0, 0.0), (0.2, 0.0, 0.0)])


def test_empty_polygon_refuses_everything():
    """영역이 비어 있으면 통과시키지 않는다 — 없는 한계를 있는 척하지 않는다."""
    empty = Workspace([], 0.0, 0.2)
    verdict = empty.check(0.2, 0.0, 0.1)
    assert not verdict.ok and "측정" in verdict.reason


def test_roundtrip_through_dict(ws):
    restored = Workspace.from_dict(ws.to_dict())
    assert restored.polygon_xy == ws.polygon_xy
    assert restored.z_min == ws.z_min and restored.z_max == ws.z_max


def test_load_returns_none_when_not_measured(tmp_path, monkeypatch):
    """측정 전에는 그럴듯한 기본 영역을 지어내지 않는다."""
    from sorting import calib
    monkeypatch.setattr(calib, "STORE",
                        calib.CalibStore(path=str(tmp_path / "c.json")))
    assert Workspace.load() is None


def test_save_and_load_via_calib_store(tmp_path, monkeypatch):
    from sorting import calib
    store = calib.CalibStore(path=str(tmp_path / "c.json"))
    monkeypatch.setattr(calib, "STORE", store)

    Workspace.from_trace(TRACE).save(note="테스트 훑기")
    loaded = Workspace.load()
    assert loaded is not None
    assert loaded.contains(0.20, 0.0, 0.10)
    assert store.get("workspace", None).source == "measured"


def test_manual_save_is_marked_manual(tmp_path, monkeypatch):
    """토크오프 추적이 안 될 때 손으로 넣은 영역은 '수동'으로 남아야 한다."""
    from sorting import calib
    store = calib.CalibStore(path=str(tmp_path / "c.json"))
    monkeypatch.setattr(calib, "STORE", store)

    Workspace.from_trace(TRACE).save(note="서보 무응답 — 조깅으로 꼭짓점 지정",
                                     manual=True)
    item = store.get("workspace", None)
    assert item.source == "manual"
    assert "무응답" in item.note


def test_summary_mentions_reach_range(ws):
    text = ws.summary
    assert "꼭짓점" in text and "z" in text
