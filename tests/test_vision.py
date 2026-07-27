# -*- coding: utf-8 -*-
"""실제로 촬영한 사진 8장으로 검출기를 검증한다.

shots/ 는 HP60C 로 책상을 내려다보며 찍은 것이라, 공 말고도 키보드·
과자봉지·스피커가 같이 찍혀 있다. 특히 왼쪽 아래 빨간 과자봉지가 좋은
오검출 시험대다 — 빨간 픽셀 덩어리지만 공이 아니다.

기대값은 사진을 눈으로 확인해 적었다.
"""
from __future__ import annotations

import glob
import os

import cv2
import numpy as np
import pytest

from sorting import config as cfg
from sorting.tracking import BallTracker
from sorting.vision import circularity, color_mask, observe

SHOTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "shots")

# 파일명 → (빨간공 수, 파란공 수)
EXPECTED = {
    "shot_000.png": (0, 2),
    "shot_001.png": (0, 2),
    "shot_002.png": (1, 1),
    "shot_003.png": (1, 1),
    "shot_004.png": (1, 1),
    "shot_005.png": (1, 1),
    "shot_006.png": (1, 1),
    "shot_007.png": (1, 1),
}


def _load(name):
    img = cv2.imread(os.path.join(SHOTS, name))
    assert img is not None, f"사진을 못 읽었다: {name}"
    return img


@pytest.mark.parametrize("name,expected", sorted(EXPECTED.items()))
def test_ball_counts(name, expected):
    """공 개수가 사진과 맞는가 — 과자봉지를 공으로 세면 여기서 걸린다."""
    scene = observe(_load(name))
    got = (len(scene.balls_of("red")), len(scene.balls_of("blue")))
    assert got == expected, f"{name}: 빨강/파랑 {got} 이지만 {expected} 이어야 한다"


@pytest.mark.parametrize("name", sorted(EXPECTED))
def test_no_region_false_positive(name):
    """이 사진들에는 bin 도 공급구역도 없다. 잡동사니를 구역으로 잡으면 안 된다."""
    scene = observe(_load(name))
    assert scene.regions == {}
    assert scene.pick_zone is None


def test_lab_color_alone_rejects_the_snack_bag():
    """★ Lab 으로 바꾼 이유: 빨간 과자봉지가 '색' 단계에서 이미 걸러진다.

    HSV 시절에는 과자봉지도 '빨강'을 통과해서, 공이 아니라는 판단을 면적과
    원형도에 전적으로 기대야 했다. Lab 에서는 a*/b* 거리가 기준색에서
    16 이상이라 색 마스크에조차 안 들어온다 — 모양 필터는 이제 보조수단이다.
    """
    img = _load("shot_004.png")
    mask = color_mask(img, cfg.RED)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = [c for c in cnts if cv2.contourArea(c) >= 150]

    assert len(blobs) == 1, (
        f"빨강 마스크에 {len(blobs)}개가 남았다 — 과자봉지가 색만으로 걸러져야 한다")

    # 남은 하나가 진짜 공인지 확인 (면적·원형도 둘 다 공다워야 한다)
    ball = blobs[0]
    assert cv2.contourArea(ball) > 800
    assert circularity(ball) >= cfg.VISION.min_circularity


def test_measured_lab_margins_hold():
    """실측한 여유(공 ≤2, 과자봉지 ≥16)가 지금도 유효한가.

    이 숫자가 무너지면 config 의 radius 를 다시 정해야 한다는 신호다.
    """
    img = _load("shot_004.png")
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    # 실측해 둔 위치 — 빨간 공과 빨간 과자봉지
    for (u, v), expect_close in (((365, 254), True), ((161, 361), False)):
        patch = lab[v - 3:v + 4, u - 3:u + 4].reshape(-1, 3).astype(float)
        a = float(np.median(patch[:, 1])) - 128.0
        b = float(np.median(patch[:, 2])) - 128.0
        dist = cfg.RED.distance_to(a, b)
        if expect_close:
            assert dist <= cfg.RED.radius * 0.5, \
                f"공이 기준색에서 {dist:.1f} — 여유가 없다"
        else:
            assert dist >= cfg.RED.radius + 4, \
                f"과자봉지가 기준색에서 {dist:.1f} — radius {cfg.RED.radius} 와 너무 가깝다"


def test_red_needs_no_hue_wraparound():
    """빨강을 두 구간으로 쪼개지 않아도 잡히는가.

    HSV 에서 빨강은 색상환 0/180 양끝에 걸쳐 범위를 둘로 나눠 OR 해야 했다.
    Lab 은 거리 판정이라 그 이음매가 아예 없다 — 이 테스트가 그걸 지킨다.
    """
    assert not hasattr(cfg.RED, "wraps"), "아직 HSV 시절 구조가 남아 있다"
    assert len(observe(_load("shot_004.png")).balls_of("red")) == 1


def test_lightness_bounds_reject_shadow_and_glare():
    """아주 어둡거나 하얗게 날아간 픽셀은 색이 맞아도 제외한다."""
    a_off = int(round(cfg.RED.a)) + 128
    b_off = int(round(cfg.RED.b)) + 128

    for lightness, should_pass in ((10, False), (128, True), (252, False)):
        lab = np.zeros((40, 40, 3), np.uint8)
        lab[:, :] = (lightness, a_off, b_off)
        bgr = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        # LAB2BGR→BGR2LAB 왕복에서 값이 조금 틀어지므로 통과 여부만 본다
        hit = np.count_nonzero(color_mask(bgr, cfg.RED))
        assert (hit > 0) == should_pass, \
            f"L={lightness} 에서 통과 여부가 기대({should_pass})와 다르다"


def test_ball_inside_bin_is_ignored(monkeypatch):
    """bin 안에 들어간 공은 목록에서 빠져야 한다.

    안 그러면 방금 놓은 공을 다시 집으러 가는 무한루프가 된다.
    """
    img = _load("shot_004.png")
    before = observe(img)
    assert len(before.balls_of("red")) == 1
    ball = before.balls_of("red")[0]

    # 그 빨간 공을 통째로 덮는 빨간 bin 이 검출된 것처럼 만든다
    from sorting import vision
    real_detect_region = vision.detect_region

    def fake(bgr, band, min_area=None, channels=None):
        if band.name == "red":
            return vision.Region("red", ball.u, ball.v,
                                 (ball.u - 60, ball.v - 60, 120, 120), 14400.0)
        return real_detect_region(bgr, band, min_area, channels)

    monkeypatch.setattr(vision, "detect_region", fake)
    after = vision.observe(img)
    assert after.balls_of("red") == [], "bin 안의 공은 대상에서 빠져야 한다"


def test_pick_zone_limits_candidates(monkeypatch):
    """공급구역이 잡히면 그 밖의 공은 무시한다."""
    img = _load("shot_004.png")
    from sorting import vision
    real_detect_region = vision.detect_region

    def fake(bgr, band, min_area=None, channels=None):
        if band.name == "orange":                 # 화면 왼쪽 절반만 공급구역
            return vision.Region("orange", 160, 240, (0, 0, 320, 480), 153600.0)
        return real_detect_region(bgr, band, min_area, channels)

    monkeypatch.setattr(vision, "detect_region", fake)
    scene = vision.observe(img)
    assert scene.pick_zone is not None
    for b in scene.balls:
        assert b.u <= 320, "공급구역 밖의 공이 후보에 남았다"


def test_tracker_needs_consecutive_frames():
    """한 프레임만 보인 공은 확정되지 않는다(노이즈 억제)."""
    img = _load("shot_004.png")
    scene = observe(img)
    tracker = BallTracker()

    assert tracker.update(scene) == [], "첫 프레임부터 확정되면 노이즈에 취약하다"
    for _ in range(cfg.TRACKING.confirm_frames - 1):
        confirmed = tracker.update(scene)
    assert len(confirmed) == len(scene.balls), "연속 검출된 공은 확정돼야 한다"


def test_tracker_forgets_disappeared_ball():
    """공이 사라지면 결국 트랙도 사라진다."""
    from sorting.vision import Scene
    img = _load("shot_004.png")
    scene = observe(img)
    tracker = BallTracker()
    for _ in range(cfg.TRACKING.confirm_frames):
        tracker.update(scene)
    assert tracker.confirmed_tracks()

    empty = Scene()
    for _ in range(cfg.TRACKING.forget_frames):
        tracker.update(empty)
    assert tracker.tracks == [], "안 보이는 공을 계속 붙들고 있으면 안 된다"


def test_every_shot_is_covered():
    """사진을 추가했는데 기대값을 안 적는 일을 막는다."""
    on_disk = {os.path.basename(p) for p in glob.glob(os.path.join(SHOTS, "*.png"))}
    assert on_disk == set(EXPECTED), f"기대값 미기재: {on_disk ^ set(EXPECTED)}"
