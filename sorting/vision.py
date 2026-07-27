# -*- coding: utf-8 -*-
"""vision.py — 공과 구역(bin·공급구역)을 같은 색 검출로 찾는다.

이 파일의 핵심 아이디어 한 줄:

    공도 구역도 "그 색 픽셀 덩어리"다. 둘을 가르는 건 색이 아니라 크기와 모양이다.

그래서 bin 을 책상 위에서 옮겨도 다음 프레임이면 새 자리에서 다시 찾아진다 —
좌표를 어디에도 적어두지 않기 때문이다. 이게 "위치가 바뀌어도 되는 분류"의 근거다.

색은 CIELAB 의 a*/b* 평면에서 기준색까지의 거리로 판정한다(config.LabColor).
실측상 이 편이 HSV 보다 여유가 크다 — shots 에서 빨간 공은 기준색에서
2 이내인데 빨간 과자봉지는 16 이상 떨어져 있어, 모양을 보기 전에 색만으로도
갈린다. HSV 로는 둘 다 '빨강'을 통과해 면적·원형도에 기대야 했다.

단독 실행하면 저장된 사진으로 검출 결과를 눈으로 확인할 수 있다:
    python -m sorting.vision shots/*.png
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from . import config as cfg


# ── 관측 결과 자료구조 ─────────────────────────────────────────────────────
@dataclass(frozen=True)
class Ball:
    color: str
    u: int
    v: int
    r: int
    area: float

    @property
    def uv(self) -> tuple[int, int]:
        return (self.u, self.v)


@dataclass(frozen=True)
class Region:
    """bin 또는 공급구역. bbox 는 (x, y, w, h)."""
    color: str
    cx: int
    cy: int
    bbox: tuple[int, int, int, int]
    area: float

    def contains(self, u: float, v: float, pad: int = 0) -> bool:
        x, y, w, h = self.bbox
        return (x - pad) <= u <= (x + w + pad) and (y - pad) <= v <= (y + h + pad)


@dataclass
class Scene:
    """한 프레임에서 읽어낸 것 전부. GUI와 파이프라인이 이 하나를 공유한다."""
    balls: list[Ball] = field(default_factory=list)
    regions: dict[str, Region] = field(default_factory=dict)
    pick_zone: Region | None = None
    frame_id: int = 0
    detect_ms: float = 0.0

    def balls_of(self, color: str) -> list[Ball]:
        return [b for b in self.balls if b.color == color]


# ── 색 마스크 ──────────────────────────────────────────────────────────────
def lab_channels(bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """BGR → (L, a*, b*). a*/b* 는 OpenCV 의 128 오프셋을 뺀 실제 값(float32).

    한 프레임에서 색을 여러 번 찾으므로, 무거운 색공간 변환은 여기 한 번만
    하고 각 색은 뺄셈만 하게 한다.
    """
    if cfg.VISION.blur_ksize:
        k = cfg.VISION.blur_ksize
        bgr = cv2.GaussianBlur(bgr, (k, k), 0)
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    lightness, a_chan, b_chan = cv2.split(lab)
    return (lightness.astype(np.float32),
            a_chan.astype(np.float32) - 128.0,
            b_chan.astype(np.float32) - 128.0)


def color_mask(bgr: np.ndarray, band: cfg.LabColor, channels=None) -> np.ndarray:
    """그 색으로 볼 픽셀만 남긴 이진 마스크 (Lab 기준색 + 반경).

    판정은 두 가지뿐이다:
      · a*/b* 평면에서 기준색까지의 거리 ≤ radius  ← 색이 맞는가
      · l_min ≤ L ≤ l_max                         ← 새까맣거나 하얗게 날아가지 않았나

    거리 하나로 판정하므로, HSV 처럼 빨강을 색상환 양끝 두 구간으로 쪼개
    OR 할 필요가 없다. 밝기(L)가 색기(a*,b*)와 분리돼 있어 그늘이 져도
    같은 색으로 남는다.

    channels 를 넘기면 Lab 변환을 재사용한다(observe 가 그렇게 쓴다).
    """
    lightness, a_chan, b_chan = channels if channels is not None else lab_channels(bgr)

    da = a_chan - band.a
    db = b_chan - band.b
    within = (da * da + db * db) <= (band.radius * band.radius)
    within &= (lightness >= band.l_min) & (lightness <= band.l_max)
    mask = within.astype(np.uint8) * 255

    k = cfg.VISION.morph_kernel
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)    # 잔점 제거
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern)   # 구멍 메우기


def circularity(contour) -> float:
    """4πA/P². 완전한 원이면 1.0, 길쭉하거나 각지면 작아진다.

    사각형 bin 테두리가 조각나 공 크기 덩어리로 잡히는 걸 막는 필터다.
    """
    peri = cv2.arcLength(contour, True)
    if peri <= 0:
        return 0.0
    return 4.0 * math.pi * cv2.contourArea(contour) / (peri * peri)


# ── 구역 검출 ──────────────────────────────────────────────────────────────
def detect_region(bgr: np.ndarray, band: cfg.LabColor,
                  min_area: float | None = None, channels=None) -> Region | None:
    """그 색에서 가장 큰 덩어리 하나를 구역으로 본다. 없으면 None.

    '가장 큰 것 하나'인 이유: 화면에 같은 색 잡동사니가 있어도 bin 은 보통
    가장 크다. min_area 하한이 공 크기 상한보다 위라 공과 절대 헷갈리지 않는다.
    """
    min_area = cfg.VISION.min_area_region if min_area is None else min_area
    cnts, _ = cv2.findContours(color_mask(bgr, band, channels),
                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None
    x, y, w, h = cv2.boundingRect(c)
    return Region(band.name, x + w // 2, y + h // 2, (x, y, w, h), area)


# ── 공 검출 ────────────────────────────────────────────────────────────────
def detect_balls(bgr: np.ndarray, band: cfg.LabColor, channels=None) -> list[Ball]:
    """그 색에서 공처럼 생긴 덩어리를 전부 찾는다(여러 개).

    면적 상·하한과 원형도를 통과해야 공으로 인정한다.
    """
    v = cfg.VISION
    cnts, _ = cv2.findContours(color_mask(bgr, band, channels),
                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out: list[Ball] = []
    for c in cnts:
        area = cv2.contourArea(c)
        if not (v.min_area_ball <= area <= v.max_area_ball):
            continue
        if circularity(c) < v.min_circularity:
            continue
        (u, cy), r = cv2.minEnclosingCircle(c)
        out.append(Ball(band.name, int(u), int(cy), int(r), area))
    return out


# ── 한 프레임 전체 관측 ────────────────────────────────────────────────────
def observe(bgr: np.ndarray, frame_id: int = 0) -> Scene:
    """프레임 하나 → Scene. 파이프라인과 GUI가 쓰는 단 하나의 진입점.

    공 채택 규칙 두 가지:
      · bin 안에 있는 공은 제외 — 이미 분류를 마친 공을 다시 집는 무한루프를 막는다.
      · 공급구역이 검출됐다면 그 안의 공만 대상 — 굴러나간 공에 팔이 따라가지 않는다.
    """
    t0 = cv2.getTickCount()
    channels = lab_channels(bgr)        # Lab 변환은 프레임당 한 번만

    regions: dict[str, Region] = {}
    for name, band in cfg.BIN_COLORS.items():
        reg = detect_region(bgr, band, channels=channels)
        if reg is not None:
            regions[name] = reg

    pick_zone = (detect_region(bgr, cfg.PICK_ZONE, channels=channels)
                 if cfg.PICK_ZONE else None)

    balls: list[Ball] = []
    for name, band in cfg.BALL_COLORS.items():
        for b in detect_balls(bgr, band, channels=channels):
            if any(r.contains(b.u, b.v) for r in regions.values()):
                continue                      # 이미 bin 안에 담긴 공
            if pick_zone is not None and not pick_zone.contains(b.u, b.v):
                continue                      # 공급구역 밖 — 대상이 아니다
            balls.append(b)

    ms = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000.0
    return Scene(balls=balls, regions=regions, pick_zone=pick_zone,
                 frame_id=frame_id, detect_ms=ms)


# ── 오버레이 ───────────────────────────────────────────────────────────────
def _label(img, text, org, color, scale=0.5):
    """검은 테두리를 깔고 그 위에 색 글씨 — 밝은 배경에서도 읽힌다."""
    for c, tk in (((0, 0, 0), 3), (color, 1)):
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, c, tk,
                    cv2.LINE_AA)


def draw_scene(bgr: np.ndarray, scene: Scene, robot_xy=None) -> np.ndarray:
    """검출 결과를 그려 넣은 새 이미지를 돌려준다.

    robot_xy: {색: (x, y)} 를 주면 구역 라벨에 로봇좌표까지 같이 찍는다.
    구역을 손으로 옮겼을 때 상자와 좌표가 따라 움직이는 게 보이는 것이
    '위치가 바뀌어도 동작함'을 눈으로 증명하는 수단이다.
    """
    out = bgr.copy()

    for name, reg in scene.regions.items():
        x, y, w, h = reg.bbox
        col = cfg.BIN_COLORS[name].draw
        cv2.rectangle(out, (x, y), (x + w, y + h), col, 2)
        txt = f"{name.upper()} BIN"
        if robot_xy and name in robot_xy:
            rx, ry = robot_xy[name]
            txt += f"  ({rx:+.3f}, {ry:+.3f})m"
        _label(out, txt, (x, max(y - 8, 14)), col)

    if scene.pick_zone is not None:
        x, y, w, h = scene.pick_zone.bbox
        col = cfg.PICK_ZONE.draw
        cv2.rectangle(out, (x, y), (x + w, y + h), col, 2)
        _label(out, "PICK ZONE", (x, max(y - 8, 14)), col)

    for b in scene.balls:
        col = cfg.BALL_COLORS[b.color].draw
        cv2.circle(out, (b.u, b.v), b.r, col, 2)
        cv2.drawMarker(out, (b.u, b.v), (0, 255, 255), cv2.MARKER_CROSS, 12, 2)
        _label(out, f"{b.color} {b.area:.0f}px", (b.u - b.r, b.v - b.r - 6), col)

    return out


# ── 사진으로 눈 확인 ───────────────────────────────────────────────────────
def _main(argv):
    import glob
    paths = [p for a in argv for p in sorted(glob.glob(a))]
    if not paths:
        raise SystemExit("사용법: python -m sorting.vision shots/*.png")
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print("읽기 실패:", p)
            continue
        sc = observe(img)
        counts = {c: len(sc.balls_of(c)) for c in cfg.BALL_COLORS}
        print(f"{p}: 공 {counts} · 구역 {sorted(sc.regions)} · "
              f"공급구역 {'O' if sc.pick_zone else 'X'} · {sc.detect_ms:.1f}ms")
        cv2.imshow("detect (아무 키=다음, q=종료)", draw_scene(img, sc))
        if (cv2.waitKey(0) & 0xFF) == ord("q"):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import sys
    _main(sys.argv[1:])
