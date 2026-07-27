# -*- coding: utf-8 -*-
"""setup_zone.py — 안전 경계선과 테이블 기준면을 한 번에 잡는다.

두 가지를 만든다:
  1. data/safety_zone.json — 로봇 작업영역 폴리곤. 이 **바깥**이 감시 띠가 된다.
  2. data/depth_baseline.npy — 빈 테이블의 깊이. 이보다 솟은 것이 '물체'다.

실행:
    python -m sorting.setup_zone

순서: ① 작업면을 비운다 → ② 로봇이 닿는 범위를 네 번 클릭 → ③ Enter 로 기준면 촬영.
로봇이 실제로 움직이는 범위보다 **조금 넉넉하게** 클릭해야 팔이 띠를 건드리지 않는다.
"""
from __future__ import annotations

import os

import cv2
import numpy as np
from hp60c_camera import CameraReader

from . import config as cfg
from .safety import Zone

BASELINE_FRAMES = 30      # 이만큼 모아 중앙값 — 깊이 센서의 프레임 노이즈를 지운다


def _hud(img, lines, color=(0, 255, 255)):
    for i, text in enumerate(lines):
        org = (8, 24 + i * 24)
        for c, tk in (((0, 0, 0), 3), (color, 1)):
            cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, tk,
                        cv2.LINE_AA)


def pick_polygon(cam) -> Zone:
    """로봇 작업영역 네 꼭짓점을 클릭받는다. u=되돌리기, q=취소."""
    win = "setup: 작업영역 4곳 클릭 / u=취소 / q=종료"
    points: list[tuple[int, int]] = []

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((x, y))

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    last = 0
    try:
        while True:
            rgb, _depth, last = cam.read_blocking(last)
            if rgb is None:
                continue
            view = rgb.copy()
            h, w = view.shape[:2]

            if len(points) >= 2:
                cv2.polylines(view, [np.array(points, np.int32)],
                              len(points) == 4, (0, 255, 0), 2)
            for i, p in enumerate(points):
                cv2.circle(view, p, 5, (0, 255, 0), -1)
                cv2.putText(view, str(i + 1), (p[0] + 8, p[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if len(points) < 4:
                _hud(view, [f"로봇이 닿는 범위의 꼭짓점 {len(points) + 1}/4 을 클릭",
                            "실제 동작범위보다 조금 넓게 — 팔이 띠에 닿으면 오작동"])
            else:
                # 바깥 띠를 어둡게 덮어 감시 영역이 어디인지 눈으로 보여준다
                band = Zone([(x / w, y / h) for x, y in points]).band_mask(view.shape)
                view[band > 0] = (view[band > 0] * 0.45).astype(np.uint8)
                _hud(view, ["어두운 곳이 감시 띠 — 여기 손이 들어오면 정지",
                            "Enter=확정  u=되돌리기  q=취소"], (0, 255, 0))

            cv2.imshow(win, view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("u") and points:
                points.pop()
            elif key == ord("q"):
                raise KeyboardInterrupt
            elif key in (13, 10) and len(points) == 4:
                return Zone([(x / w, y / h) for x, y in points])
    finally:
        cv2.destroyWindow(win)


def capture_baseline(cam) -> np.ndarray:
    """빈 테이블 깊이를 여러 장 모아 중앙값으로 굳힌다.

    중앙값을 쓰는 이유: 깊이 센서는 프레임마다 몇 픽셀씩 0(무효)이 튄다.
    평균을 내면 그 0 이 기준면을 끌어내려 멀쩡한 테이블이 '솟은 것'으로 보인다.
    """
    print(f"\n작업면에서 손과 물건을 모두 치우고 Enter (프레임 {BASELINE_FRAMES}장 촬영)")
    input("> ")

    frames: list[np.ndarray] = []
    last = 0
    while len(frames) < BASELINE_FRAMES:
        _rgb, depth, last = cam.read_blocking(last)
        if depth is not None:
            frames.append(depth.astype(np.float32))
        print(f"\r  {len(frames)}/{BASELINE_FRAMES}", end="", flush=True)
    print()

    stack = np.stack(frames)
    stack[stack == 0] = np.nan                      # 무효 픽셀은 계산에서 뺀다
    with np.errstate(invalid="ignore"):
        median = np.nanmedian(stack, axis=0)
    median = np.nan_to_num(median, nan=0.0)

    valid = np.count_nonzero(median)
    total = median.size
    print(f"  유효 픽셀 {valid}/{total} ({100 * valid / total:.0f}%)")
    if valid < total * 0.5:
        print("  ⚠ 유효 픽셀이 적습니다 — 카메라가 너무 가깝거나 표면이 반사성일 수 있습니다.")
    return median.astype(np.uint16)


def main() -> None:
    os.makedirs(cfg.DATA, exist_ok=True)
    try:
        with CameraReader() as cam:
            zone = pick_polygon(cam)
            path = zone.save()
            print("저장:", path)

            baseline = capture_baseline(cam)
            np.save(cfg.BASELINE_FILE, baseline)
            print("저장:", cfg.BASELINE_FILE)
    except KeyboardInterrupt:
        print("\n취소 — 저장하지 않았습니다.")
        return
    finally:
        cv2.destroyAllWindows()

    print("\n확인:  python -m sorting.safety")


if __name__ == "__main__":
    main()
