# -*- coding: utf-8 -*-
"""딜레이 보호장치 검증.

워치독은 타임스탬프만 받는 순수 로직이라 로봇도 카메라도 없이 전부 검증된다.
여기서 지키려는 것:
  · 나빠지면 즉시 올라가고, 좋아지면 천천히 내려온다(깜빡임에 안 흔들린다)
  · 각 단계가 실제 행동(새 동작 금지·감속·정지)으로 이어진다
"""
from __future__ import annotations

import pytest

from sorting.watchdog import (CAMERA, ROBOT, LatencyWatchdog, Level,
                              Thresholds, measure_thresholds)

TH = Thresholds(warn_ms=300, hold_ms=800, stop_ms=1500, recover_samples=3)


@pytest.fixture
def wd():
    return LatencyWatchdog(TH)


@pytest.mark.parametrize("ms,expected", [
    (0, Level.NORMAL), (299, Level.NORMAL),
    (300, Level.WARN), (799, Level.WARN),
    (800, Level.HOLD), (1499, Level.HOLD),
    (1500, Level.STOP), (5000, Level.STOP),
])
def test_level_boundaries(wd, ms, expected):
    assert wd.report(CAMERA, ms) is expected


def test_worsening_is_immediate(wd):
    """나빠질 때는 기다리지 않는다 — 여러 단계도 한 번에 뛴다."""
    wd.report(CAMERA, 10)
    assert wd.level is Level.NORMAL
    assert wd.report(CAMERA, 2000) is Level.STOP, "위험 반응이 늦으면 안 된다"


def test_recovery_needs_consecutive_good_samples(wd):
    """한 번 좋아졌다고 바로 풀리면 안 된다."""
    wd.report(CAMERA, 2000)
    assert wd.level is Level.STOP

    for _ in range(TH.recover_samples - 1):
        wd.report(CAMERA, 10)
    assert wd.level is Level.STOP, "회복 표본이 다 차기 전에 풀렸다"

    wd.report(CAMERA, 10)
    assert wd.level is Level.HOLD, "회복은 한 단계씩만 내려와야 한다"


def test_recovery_steps_down_one_level_at_a_time(wd):
    """정지에서 정상으로 곧장 뛰지 않는다.

    900→400→200ms 처럼 아직 나쁜 값이 섞인 구간에서 '개선 3회'로 세어
    정지에서 정상으로 건너뛰면, 여전히 불안정한데 팔이 다시 출발한다.
    """
    wd.report(CAMERA, 2000)
    seen = [wd.level]
    for _ in range(TH.recover_samples * 3):
        wd.report(CAMERA, 5)
        seen.append(wd.level)

    assert Level.HOLD in seen and Level.WARN in seen, f"단계를 건너뛰었다: {seen}"
    assert seen[-1] is Level.NORMAL
    assert seen.index(Level.HOLD) < seen.index(Level.WARN) < seen.index(Level.NORMAL)


def test_flapping_does_not_release(wd):
    """좋았다 나빴다를 반복하면 계속 잡고 있어야 한다."""
    wd.report(CAMERA, 2000)
    for _ in range(10):
        wd.report(CAMERA, 5)        # 좋아짐
        wd.report(CAMERA, 2000)     # 다시 나빠짐
    assert wd.level is Level.STOP, "깜빡이는 동안 풀려버렸다"


def test_actions_follow_level(wd):
    """단계가 말뿐이 아니라 실제 행동으로 이어지는가."""
    wd.report(CAMERA, 10)
    assert wd.may_start_motion() and not wd.should_abort()
    assert wd.speed_scale() == 1.0

    wd.report(CAMERA, 400)                       # WARN
    assert wd.may_start_motion(), "경고 단계에서는 아직 움직일 수 있다"
    assert wd.speed_scale() < 1.0, "경고 단계면 감속해야 한다"
    assert not wd.should_abort()

    wd.report(CAMERA, 1000)                      # HOLD
    assert not wd.may_start_motion(), "보류 단계에서 새 동작을 시작하면 안 된다"
    assert not wd.should_abort(), "보류는 진행 중인 동작까지 끊지는 않는다"

    wd.report(CAMERA, 2000)                      # STOP
    assert wd.should_abort(), "정지 단계는 이동 중에도 끊어야 한다"
    assert wd.speed_scale() == 0.0


def test_worst_channel_is_reported(wd):
    """무엇이 느린지 말해줘야 원인을 찾을 수 있다."""
    wd.report(CAMERA, 100)
    wd.report(ROBOT, 900)
    name, ms = wd.worst_channel()
    assert name == ROBOT and ms == pytest.approx(900)
    assert wd.level is Level.HOLD


def test_injected_delay_drives_the_same_path(wd):
    """결함 주입이 실제 지연과 같은 경로를 타는가 (데모가 진짜를 보여주는지)."""
    wd.report(CAMERA, 10)
    assert wd.level is Level.NORMAL

    wd.inject(2000)
    wd.report(CAMERA, 10)
    assert wd.level is Level.STOP, "주입한 지연이 판정에 반영되지 않았다"
    assert wd.should_abort()

    wd.inject(0)
    for _ in range(TH.recover_samples * 3):
        wd.report(CAMERA, 10)
    assert wd.level is Level.NORMAL


def test_frame_age_uses_capture_time(wd):
    """프레임이 '언제 찍혔는지'로 나이를 재는가."""
    import time
    stale = time.monotonic() - 2.0          # 2초 묵은 프레임
    assert wd.report_frame_age(stale) is Level.STOP


def test_thresholds_can_be_measured():
    """임계값을 손으로 정하지 않고 실제 분포에서 뽑을 수 있는가."""
    fast = measure_thresholds([20.0] * 50 + [35.0] * 5)
    slow = measure_thresholds([180.0] * 50 + [400.0] * 5)

    assert slow.warn_ms > fast.warn_ms, "느린 장비면 임계도 높아져야 한다"
    assert fast.warn_ms < fast.hold_ms < fast.stop_ms

    with pytest.raises(ValueError):
        measure_thresholds([])


def test_reset_clears_state(wd):
    wd.report(CAMERA, 2000)
    wd.reset()
    assert wd.level is Level.NORMAL and not wd.should_abort()


def test_status_text_mentions_injection(wd):
    wd.inject(500)
    wd.report(CAMERA, 50)
    assert "주입" in wd.status_text(), "주입 중임을 화면에 밝혀야 진짜 고장과 구분된다"
