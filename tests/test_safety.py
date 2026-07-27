# -*- coding: utf-8 -*-
"""안전 감시를 합성 깊이 프레임으로 검증한다.

카메라 앞에 실제로 손을 넣어볼 수 없으니, "테이블은 1000mm, 손은 900mm" 같은
깊이 배열을 직접 만들어 넣는다. 확인하려는 건 결국 세 가지다:
  · 띠 안의 물체만 잡고, 로봇이 도는 안쪽은 무시하는가
  · 깜빡임 한 번에 로봇이 섰다 갔다 하지 않는가(히스테리시스)
  · 설정 파일이 없을 때 '안전한 척' 하지 않는가
"""
from __future__ import annotations

import numpy as np
import pytest

from sorting import config as cfg
from sorting.safety import SafetyMonitor, Zone

H, W = 120, 160
TABLE_MM = 1000

# 화면 가운데 절반이 로봇 작업영역 → 나머지 바깥이 감시 띠
ZONE = Zone([(0.25, 0.25), (0.75, 0.25), (0.75, 0.75), (0.25, 0.75)])


def _baseline() -> np.ndarray:
    return np.full((H, W), TABLE_MM, dtype=np.uint16)


def _depth_with_object(rows, cols, height_mm=100) -> np.ndarray:
    """지정한 영역이 테이블보다 height_mm 만큼 솟아 있는 깊이 프레임."""
    d = np.full((H, W), TABLE_MM, dtype=np.uint16)
    d[rows, cols] = TABLE_MM - height_mm
    return d


@pytest.fixture
def monitor():
    return SafetyMonitor(ZONE, _baseline())


def test_reports_disabled_without_calibration():
    """경계선·기준면이 없으면 감시가 꺼진 것으로 보고해야 한다.

    조용히 '침입 없음'만 돌려주면, 안전장치가 없는데 있는 줄 알고 쓰게 된다.
    """
    assert SafetyMonitor(None, None).enabled is False
    assert SafetyMonitor(ZONE, None).enabled is False
    assert SafetyMonitor(None, _baseline()).enabled is False
    assert SafetyMonitor(ZONE, _baseline()).enabled is True


def test_object_inside_robot_area_is_ignored(monitor):
    """로봇팔이 도는 안쪽에서 솟은 것은 침입이 아니다 — 그게 바로 팔이다."""
    depth = _depth_with_object(slice(40, 80), slice(50, 110))   # 가운데 = 작업영역
    for _ in range(cfg.SAFETY.enter_frames + 2):
        assert monitor.update(depth) is False
    assert monitor.last_pixels == 0


def test_object_in_band_triggers(monitor):
    """띠 안에 손이 들어오면 잡는다."""
    depth = _depth_with_object(slice(0, 25), slice(0, 100))     # 위쪽 = 띠
    for _ in range(cfg.SAFETY.enter_frames):
        monitor.update(depth)
    assert monitor.intruded is True
    assert monitor.last_pixels >= monitor.threshold_px


def test_small_object_does_not_trigger(monitor):
    """먼지나 굴러간 공 정도의 작은 덩어리로는 멈추지 않는다."""
    depth = _depth_with_object(slice(0, 3), slice(0, 5))
    for _ in range(cfg.SAFETY.enter_frames + 2):
        monitor.update(depth)
    assert monitor.intruded is False


def test_flat_object_below_threshold_ignored(monitor):
    """테이블에 거의 붙은 얇은 것(종이 등)은 무시한다."""
    depth = _depth_with_object(slice(0, 25), slice(0, 100),
                               height_mm=cfg.SAFETY.intrusion_mm - 10)
    for _ in range(cfg.SAFETY.enter_frames + 2):
        monitor.update(depth)
    assert monitor.intruded is False


def test_clearing_is_slower_than_triggering(monitor):
    """빠지는 판정이 들어오는 판정보다 보수적이어야 한다.

    한 프레임 깜빡였다고 로봇이 다시 움직이기 시작하면, 그 사이 손이 아직
    있는데 팔이 출발하는 상황이 생긴다.
    """
    hand = _depth_with_object(slice(0, 25), slice(0, 100))
    clear = np.full((H, W), TABLE_MM, dtype=np.uint16)

    for _ in range(cfg.SAFETY.enter_frames):
        monitor.update(hand)
    assert monitor.intruded

    for _ in range(cfg.SAFETY.clear_frames - 1):
        monitor.update(clear)
    assert monitor.intruded, "clear_frames 를 다 채우기 전에 풀렸다"

    monitor.update(clear)
    assert monitor.intruded is False


def test_single_clear_frame_does_not_release(monitor):
    """손이 있는 중에 깊이가 한 프레임 튀어도 정지 상태를 유지한다."""
    hand = _depth_with_object(slice(0, 25), slice(0, 100))
    clear = np.full((H, W), TABLE_MM, dtype=np.uint16)

    for _ in range(cfg.SAFETY.enter_frames):
        monitor.update(hand)
    monitor.update(clear)          # 센서 노이즈로 한 프레임 비었다
    assert monitor.intruded is True


def test_invalid_depth_pixels_are_not_intrusion(monitor):
    """깊이 0(측정 실패)은 '아주 가까운 물체'가 아니라 무효다."""
    depth = np.full((H, W), TABLE_MM, dtype=np.uint16)
    depth[0:30, :] = 0
    for _ in range(cfg.SAFETY.enter_frames + 2):
        monitor.update(depth)
    assert monitor.intruded is False


def test_depth_resolution_may_differ_from_baseline(monitor):
    """깊이와 기준면 해상도가 달라도 동작한다(HP60C 는 RGB/깊이 렌즈가 다르다)."""
    small = np.full((H // 2, W // 2), TABLE_MM, dtype=np.uint16)
    small[0:12, 0:50] = TABLE_MM - 100          # 절반 해상도에서 띠의 약 2%
    for _ in range(cfg.SAFETY.enter_frames):
        monitor.update(small)
    assert monitor.intruded is True


def test_sensitivity_survives_resolution_change():
    """같은 크기의 손은 해상도가 달라도 똑같이 잡혀야 한다.

    임계값을 절대 픽셀 수로 두면 640x480 에서 맞춘 감도가 320x240 에서는
    4배 둔해진다 — 안전장치가 해상도에 따라 조용히 약해지는 셈이다.
    그래서 띠 면적 대비 비율로 판정한다.
    """
    def hand_covering(frac, shape):
        """띠 면적의 frac 만큼을 차지하는 물체가 있는 깊이 프레임."""
        h, w = shape
        depth = np.full(shape, TABLE_MM, dtype=np.uint16)
        band_px = int(np.count_nonzero(ZONE.band_mask(shape)))
        rows = max(1, int(band_px * frac / w))
        depth[0:rows, :] = TABLE_MM - 100        # 맨 윗줄들은 전부 띠 안이다
        return depth

    big = SafetyMonitor(ZONE, np.full((480, 640), TABLE_MM, dtype=np.uint16))
    small = SafetyMonitor(ZONE, np.full((120, 160), TABLE_MM, dtype=np.uint16))

    for mon, shape in ((big, (480, 640)), (small, (120, 160))):
        mon.reset()
        for _ in range(cfg.SAFETY.enter_frames + 2):
            mon.update(hand_covering(0.003, shape))      # 임계(1%) 아래
        assert mon.intruded is False, f"{shape}: 작은 물체에 반응했다"

        mon.reset()
        for _ in range(cfg.SAFETY.enter_frames):
            mon.update(hand_covering(0.05, shape))       # 임계 위
        assert mon.intruded is True, f"{shape}: 손 크기 물체를 놓쳤다"


def test_zone_roundtrip(tmp_path):
    """저장했다 읽으면 같은 폴리곤이어야 한다."""
    path = tmp_path / "zone.json"
    ZONE.save(str(path))
    loaded = Zone.load(str(path))
    assert loaded is not None
    assert loaded.polygon_norm == ZONE.polygon_norm
    assert np.array_equal(loaded.band_mask((H, W)), ZONE.band_mask((H, W)))


def test_zone_load_missing_returns_none(tmp_path):
    assert Zone.load(str(tmp_path / "nope.json")) is None


def test_band_mask_covers_outside_only():
    """띠 마스크가 작업영역 바깥만 덮는지 직접 확인."""
    mask = ZONE.band_mask((H, W))
    assert mask[H // 2, W // 2] == 0, "작업영역 한가운데가 감시 대상이 되면 안 된다"
    assert mask[0, 0] > 0, "모서리는 감시 대상이어야 한다"
