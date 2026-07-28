# -*- coding: utf-8 -*-
"""02_detect.py — Lab로 빨강/파랑 공 검출 → 공의 픽셀(u,v).

카메라:        python 02_detect.py
저장 사진으로: python 02_detect.py shots/shot_000.png ...   (카메라 없이 개발)
키: q=종료

HSV 대신 CIELAB 을 쓰는 이유: L(밝기)과 a*/b*(색기)가 분리돼 있어 그늘이 져도
색 판정이 거의 안 흔들린다. 빨강도 색상환 이음매(0·180)가 없어 두 구간으로
쪼개 OR 할 필요가 없다 — a*/b* 평면에서 기준색까지의 거리가 radius 이내인지만 본다.
"""

import sys

import cv2
import numpy as np

# ── TODO ①: 빨강 Lab 기준값 ───────────────────────────────────────────────
# 할 일 : hsv_tuner.py 로 빨간 공을 클릭해 기준색을 잡고, radius/Lmin/Lmax로
#         마스크가 공만 하얗게 남을 때까지 다듬은 뒤 p 키로 출력된 값을 옮겨 적는다.
# 키워드: hsv_tuner.py, 클릭=샘플, p 키, (a, b, radius)
RED = (56, 22, 43)  # (a, b, radius) — 실측값
# ──────────────────────────────────────────────────────────────────────────

BLUE = (-13, -49, 41)  # (a, b, radius) — 실측값

L_MIN, L_MAX = 25, 245  # 그림자/반사 컷 — 색 구분은 a*/b* 가 하므로 넉넉하게 둔다
MIN_AREA = 150
DRAW = {"red": (0, 0, 255), "blue": (255, 0, 0)}


def lab_channels(bgr):
    """BGR → (L, a*, b*). a*/b* 는 OpenCV 의 128 오프셋을 뺀 실제 값(float32)."""
    # ── TODO ②: BGR → Lab 변환 ────────────────────────────────────────────
    # 할 일 : 카메라 이미지(bgr, BGR 색공간)를 Lab 로 변환해 lab 에 담는다.
    # 키워드: cv2.cvtColor, cv2.COLOR_BGR2LAB
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    # ──────────────────────────────────────────────────────────────────────
    L, a, b = cv2.split(lab)
    return (
        L.astype(np.float32),
        a.astype(np.float32) - 128.0,
        b.astype(np.float32) - 128.0,
    )


def color_mask(bgr, ref, channels=None):
    """Lab 기준색 ref=(a,b,radius) → 흑백 마스크(공=흰색).
    판정은 하나뿐: a*/b* 평면에서 기준색까지의 거리가 radius 이내인가."""
    L, a_chan, b_chan = channels if channels is not None else lab_channels(bgr)
    a_ref, b_ref, radius = ref
    da, db = a_chan - a_ref, b_chan - b_ref
    within = (da * da + db * db <= radius * radius) & (L >= L_MIN) & (L <= L_MAX)
    mask = within.astype(np.uint8) * 255
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)  # 잔점 제거
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern)  # 구멍 메우기


def detect_ball(bgr, ref, channels=None):
    """색 마스크에서 가장 큰 덩어리 → 중심 픽셀(u,v)·반지름 r. 없으면 None."""
    cnts, _ = cv2.findContours(
        color_mask(bgr, ref, channels), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    (u, v), r = cv2.minEnclosingCircle(c)
    return int(u), int(v), int(r)


def annotate(bgr):
    channels = lab_channels(bgr)  # 프레임당 한 번만 변환, 두 색이 재사용
    for color, ref in (("red", RED), ("blue", BLUE)):
        det = detect_ball(bgr, ref, channels)
        if det:
            u, v, r = det
            cv2.circle(bgr, (u, v), r, DRAW[color], 2)
            cv2.drawMarker(bgr, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 14, 2)
            cv2.putText(
                bgr,
                color,
                (u - r, v - r - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                DRAW[color],
                2,
            )
    return bgr


def main():
    if RED == (0, 0, 0) or BLUE == (0, 0, 0):
        raise SystemExit(
            "TODO ① 먼저: hsv_tuner.py 로 구한 값을 파일 상단 RED / BLUE 에 채우세요."
        )

    if len(sys.argv) > 1:  # 저장 사진으로(카메라 없이)
        for path in sys.argv[1:]:
            img = cv2.imread(path)
            if img is None:
                print("못 읽음:", path)
                continue
            cv2.imshow(path, annotate(img))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    from hp60c_camera import CameraReader  # 카메라 모드

    print("빨강/파랑 공 검출 — q 종료")
    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _depth, last = cam.read_blocking(last)
            if rgb is None:
                continue
            cv2.imshow("detect (q=quit)", annotate(rgb))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
