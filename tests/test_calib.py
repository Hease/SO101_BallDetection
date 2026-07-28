# -*- coding: utf-8 -*-
"""실측값 저장소 검증.

여기서 지키려는 것은 하나다: **어떤 값이든 출처를 잃지 않는다.**
측정값인지, 사람이 손으로 넣은 값인지, 아무도 안 잰 기본값인지가 항상 구분돼야
"이 숫자 왜 이래?"에 답할 수 있다.
"""
from __future__ import annotations

import json

import pytest

from sorting.calib import DEFAULT, MANUAL, MEASURED, CalibStore, Value


@pytest.fixture
def store(tmp_path):
    return CalibStore(path=str(tmp_path / "calibration.json"))


def test_unknown_key_falls_back_to_default(store):
    """안 잰 값은 기본값을 주되, 기본값이라는 사실을 숨기지 않는다."""
    item = store.get("robot.z_grasp", 0.005)
    assert item.value == 0.005
    assert item.source == DEFAULT
    assert item.is_default


def test_measured_beats_default(store):
    store.set_measured("robot.z_grasp", 0.004)
    item = store.get("robot.z_grasp", 0.005)
    assert item.value == 0.004
    assert item.source == MEASURED
    assert not item.is_default


def test_manual_overrides_measured(store):
    """자동 측정이 틀렸을 때 사람이 덮어쓸 수 있어야 한다.

    하드웨어가 이상해서 측정이 엉뚱하게 나오는 상황이 실제로 있다.
    그때 막히지 않고 손으로 고칠 수 있는 것이 이 저장소의 존재 이유다.
    """
    store.set_measured("robot.grip_empty_frac", 3.0)
    store.set_manual("robot.grip_empty_frac", 8.0, note="서보6 위치읽기 불량")

    item = store.get("robot.grip_empty_frac", 5.0)
    assert item.value == 8.0
    assert item.source == MANUAL
    assert "서보6" in item.note


def test_manual_note_survives_reload(tmp_path):
    """수동으로 넣은 이유가 파일에 남아야 나중에 설명할 수 있다."""
    path = str(tmp_path / "c.json")
    CalibStore(path=path).set_manual("robot.z_grasp", 0.009, note="자로 측정")

    reloaded = CalibStore(path=path).get("robot.z_grasp", 0.0)
    assert reloaded.value == 0.009
    assert reloaded.source == MANUAL
    assert reloaded.note == "자로 측정"
    assert reloaded.at > 0


def test_clear_returns_to_default(store):
    store.set_manual("vision.orange", [1, 2, 3])
    assert store.clear("vision.orange") is True
    assert store.get("vision.orange", "fallback").source == DEFAULT
    assert store.clear("vision.orange") is False       # 두 번째는 지울 게 없다


def test_roundtrip_preserves_types(tmp_path):
    """숫자·리스트·튜플 모두 저장했다 읽어도 값이 유지되는가."""
    path = str(tmp_path / "c.json")
    a = CalibStore(path=path)
    a.set_measured("num", 0.0123)
    a.set_measured("lst", [[0.1, 0.2], [0.3, 0.4]])
    a.set_measured("txt", "left")

    b = CalibStore(path=path)
    assert b.value("num", None) == pytest.approx(0.0123)
    assert b.value("lst", None) == [[0.1, 0.2], [0.3, 0.4]]
    assert b.value("txt", None) == "left"


def test_missing_lists_what_still_needs_measuring(store):
    """데모 전에 '무엇을 아직 안 쟀나'를 물어볼 수 있어야 한다."""
    before = {k for k, _d, _h in store.missing()}
    assert "workspace" in before, "작업영역은 실측 대상 목록에 있어야 한다"
    assert "vision.orange" in before

    store.set_measured("workspace", {"polygon_xy": []})
    after = {k for k, _d, _h in store.missing()}
    assert "workspace" not in after
    assert "vision.orange" in after


def test_missing_tells_how_to_measure(store):
    """무엇이 빠졌는지만 알려주면 부족하다 — 재는 명령까지 알려줘야 한다."""
    for _key, _desc, how in store.missing():
        assert how.startswith("python -m sorting."), f"재는 방법이 비었다: {how}"


def test_report_includes_unmeasured_items(store):
    keys = {k for k, _d, _v in store.report()}
    assert "robot.z_grasp" in keys and "workspace" in keys


def test_corrupt_file_does_not_crash(tmp_path, capsys):
    """캘리브레이션 파일이 깨져도 프로그램은 기본값으로 계속 가야 한다.

    데모 직전에 파일 하나 때문에 아무것도 안 뜨는 상황을 막는다.
    """
    path = tmp_path / "c.json"
    path.write_text("{ 이건 JSON 이 아니다", encoding="utf-8")

    store = CalibStore(path=str(path))
    assert store.get("robot.z_grasp", 0.005).value == 0.005
    assert "읽기 실패" in capsys.readouterr().out


def test_save_is_atomic(tmp_path):
    """쓰다 죽어도 기존 파일이 남도록 임시파일→교체 방식이어야 한다."""
    path = tmp_path / "c.json"
    store = CalibStore(path=str(path))
    store.set_measured("a", 1)
    store.set_measured("b", 2)

    assert not (tmp_path / "c.json.tmp").exists(), "임시파일이 남아 있으면 안 된다"
    data = json.loads(path.read_text(encoding="utf-8"))
    assert set(data["values"]) == {"a", "b"}


def test_value_age_text():
    import time as _t
    assert Value(1, MEASURED, _t.time()).age_text().endswith("시간 전")
    assert Value(1, MEASURED, _t.time() - 3 * 86400).age_text() == "3일 전"
    assert Value(1, DEFAULT).age_text() == ""
