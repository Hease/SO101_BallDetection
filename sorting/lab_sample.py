# -*- coding: utf-8 -*-
"""lab_sample.py — 공이나 구역을 클릭해 Lab 기준색을 잰다.

이게 이 시스템의 색 캘리브레이션 도구다. HSV 슬라이더 여섯 개를 맞추는 대신,
**대상을 클릭하면 config.py 에 그대로 붙여넣을 한 줄을 뱉는다.**

    python -m sorting.lab_sample                    # 카메라로
    python -m sorting.lab_sample shots/*.png # 저장된 사진으로

클릭하면 그 지점 주변 픽셀의 중앙값 a*/b* 를 재고, 현재 설정된 색들과 얼마나
떨어져 있는지도 같이 알려준다. 거리가 가까우면 서로 헷갈릴 수 있다는 뜻이다.

키: 클릭=측정 · r=측정 초기화 · s=요약 출력 · n=다음 사진 · q=종료
"""
from __future__ import annotations

import glob
import sys

import cv2
import numpy as np

from . import config as cfg
from .vision import lab_channels

PATCH = 7           # 클릭 지점 주변 이만큼의 정사각형에서 색을 잰다(홀수)
MIN_MARGIN = 8.0    # 다른 색과 이 정도는 떨어져 있어야 안심할 수 있다


def sample_at(channels, u: int, v: int, patch: int = PATCH):
    """(u,v) 주변 패치의 (L, a*, b*) 중앙값과 흔들림 폭."""
    lightness, a_chan, b_chan = channels
    h, w = lightness.shape
    half = patch // 2
    y0, y1 = max(0, v - half), min(h, v + half + 1)
    x0, x1 = max(0, u - half), min(w, u + half + 1)

    cut = [c[y0:y1, x0:x1].ravel() for c in (lightness, a_chan, b_chan)]
    med = [float(np.median(c)) for c in cut]
    spread = [float(c.max() - c.min()) for c in cut]
    return med, spread


def nearest_colors(a: float, b: float) -> list[tuple[str, float]]:
    """설정된 모든 색까지의 a*/b* 거리를 가까운 순으로."""
    known = dict(cfg.BALL_COLORS)
    known.update(cfg.BIN_COLORS)
    if cfg.PICK_ZONE is not None:
        known[cfg.PICK_ZONE.name] = cfg.PICK_ZONE
    return sorted(((c.name, c.distance_to(a, b)) for c in known.values()),
                  key=lambda t: t[1])


def suggest_line(name: str, samples: list[tuple[float, float]]) -> str:
    """모은 측정값 → config.py 에 붙여넣을 한 줄.

    반경은 '모은 점들이 중심에서 벗어난 최대 거리'에 여유를 얹어 정한다.
    한 점만 쟀으면 벗어남이 0 이라 기본값을 쓴다.
    """
    arr = np.asarray(samples, dtype=float)
    center = np.median(arr, axis=0)
    if len(arr) > 1:
        spread = float(np.max(np.linalg.norm(arr - center, axis=1)))
        radius = max(8.0, round(spread * 1.8 + 4.0))
    else:
        radius = 12.0
    return (f'{name.upper()} = LabColor("{name}", a={center[0]:+.1f}, '
            f'b={center[1]:+.1f}, radius={radius:.0f}, draw=(0, 0, 255))')


class Sampler:
    """클릭을 모아 요약을 만들어 준다."""

    def __init__(self):
        self.samples: list[tuple[float, float]] = []
        self.last: tuple | None = None

    def take(self, channels, u: int, v: int) -> None:
        med, spread = sample_at(channels, u, v)
        lightness, a, b = med
        self.samples.append((a, b))
        self.last = (u, v, lightness, a, b, spread)

        print(f"\n[{len(self.samples)}] 클릭 ({u},{v})")
        print(f"    L={lightness:5.1f}  a*={a:+6.1f}  b*={b:+6.1f}"
              f"   (패치 내 흔들림 L={spread[0]:.0f} a={spread[1]:.0f} b={spread[2]:.0f})")
        near = nearest_colors(a, b)
        for cname, dist in near[:3]:
            flag = "  ← 이 색으로 잡힘" if dist <= _radius_of(cname) else ""
            print(f"    {cname:<7} 까지 거리 {dist:6.1f}{flag}")
        if len(near) > 1 and near[1][1] < MIN_MARGIN:
            print(f"    ⚠ '{near[0][0]}' 와 '{near[1][0]}' 가 너무 가깝습니다 "
                  f"({near[1][1]:.1f}) — 두 색이 헷갈릴 수 있어요.")

    def summary(self) -> None:
        if not self.samples:
            print("\n측정한 점이 없습니다. 대상을 클릭하세요.")
            return
        arr = np.asarray(self.samples)
        center = np.median(arr, axis=0)
        dists = np.linalg.norm(arr - center, axis=1)
        print(f"\n{'=' * 62}")
        print(f"측정 {len(self.samples)}점  →  중심 a*={center[0]:+.1f} b*={center[1]:+.1f}")
        print(f"중심에서 벗어난 정도: 최대 {dists.max():.1f}, 평균 {dists.mean():.1f}")
        print("\nconfig.py 에 붙여넣을 줄 (이름·draw 색은 알맞게 고치세요):")
        print("   ", suggest_line("mycolor", self.samples))
        print(f"{'=' * 62}\n")

    def reset(self) -> None:
        self.samples.clear()
        self.last = None
        print("\n측정값을 비웠습니다.")


def _radius_of(name: str) -> float:
    known = dict(cfg.BALL_COLORS)
    known.update(cfg.BIN_COLORS)
    if cfg.PICK_ZONE is not None:
        known[cfg.PICK_ZONE.name] = cfg.PICK_ZONE
    color = known.get(name)
    return color.radius if color else 0.0


def _draw_hud(view, sampler: Sampler) -> None:
    lines = ["클릭=측정  r=초기화  s=요약  n=다음  q=종료",
             f"측정 {len(sampler.samples)}점"]
    if sampler.last is not None:
        u, v, lightness, a, b, _spread = sampler.last
        lines.append(f"L={lightness:.0f} a*={a:+.1f} b*={b:+.1f}")
        cv2.drawMarker(view, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
        cv2.rectangle(view, (u - PATCH // 2, v - PATCH // 2),
                      (u + PATCH // 2, v + PATCH // 2), (0, 255, 255), 1)
    for i, text in enumerate(lines):
        org = (8, 24 + i * 24)
        for color, thick in (((0, 0, 0), 3), ((0, 255, 255), 1)):
            cv2.putText(view, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        color, thick, cv2.LINE_AA)


def _run(frames, sampler: Sampler, live: bool) -> None:
    """frames: (이름, 이미지) 를 주는 반복자. live 면 계속 새 프레임을 받는다."""
    win = "lab_sample — 클릭해서 색 측정"
    click: dict[str, tuple[int, int]] = {}

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click["uv"] = (x, y)

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    try:
        for _name, image in frames:
            channels = lab_channels(image)
            while True:
                view = image.copy()
                _draw_hud(view, sampler)
                cv2.imshow(win, view)
                key = cv2.waitKey(30) & 0xFF

                if "uv" in click:
                    u, v = click.pop("uv")
                    sampler.take(channels, u, v)
                if key == ord("q"):
                    return
                if key == ord("r"):
                    sampler.reset()
                if key == ord("s"):
                    sampler.summary()
                if key == ord("n") and not live:
                    break
                if live:
                    break          # 라이브면 매번 새 프레임으로
    finally:
        cv2.destroyAllWindows()


def main(argv: list[str]) -> None:
    sampler = Sampler()
    print(__doc__)
    print("현재 설정된 색:")
    for color in list(cfg.BALL_COLORS.values()) + (
            [cfg.PICK_ZONE] if cfg.PICK_ZONE else []):
        print(f"  {color.name:<7} a*={color.a:+6.1f} b*={color.b:+6.1f} "
              f"radius={color.radius:.0f}")

    paths = [p for arg in argv for p in sorted(glob.glob(arg))]
    if paths:
        def frames():
            for path in paths:
                image = cv2.imread(path)
                if image is None:
                    print("읽기 실패:", path)
                    continue
                print(f"\n--- {path} ---")
                yield path, image
        _run(frames(), sampler, live=False)
    else:
        from hp60c_camera import CameraReader

        def frames():
            with CameraReader() as cam:
                last = 0
                while True:
                    rgb, _depth, last = cam.read_blocking(last)
                    if rgb is not None:
                        yield "camera", rgb
        _run(frames(), sampler, live=True)

    sampler.summary()


if __name__ == "__main__":
    main(sys.argv[1:])
