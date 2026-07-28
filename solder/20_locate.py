# -*- coding: utf-8 -*-
"""20_locate.py — 탑다운으로 인두기(샤프트 색 밴드)의 위치를 검출한다 (모션 없음).

'카메라 위치검출(방향 고정)' 픽업의 1단계. 인두기는 일정한 방향으로 놓이고 위치만
바뀐다고 가정한다. 샤프트에 붙인 색 테이프 밴드를 Lab 로 검출해:
  · 마커 픽셀 (u,v)  → H(data/H.npy)로 로봇 xy 로 변환
  · 마커 덩어리의 긴축 각도(놓인 방향 확인용 — 어긋나면 경고)
을 화면에 그린다. 로봇은 움직이지 않는다(안전). 21_pick 이 이 xy 로 파지한다.

먼저 카메라가 움직였으면 03_calib_auto.py 로 H 를 다시 만들 것.
마커 색은 아래 Lab 상수로 맞춘다 — 화면을 클릭하면 그 점의 Lab 값을 찍어 준다.
키: q=종료
"""
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for p in (_ROOT, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from hw import make_camera             # 실물/mock 자동

H_PATH = os.path.join(_ROOT, "data", "H.npy")

# ── 인두기 마커(색 밴드) Lab 범위 (lo, hi). 클릭 샘플로 맞춘다 ────────────────
# OpenCV 8bit Lab: a,b 는 128 이 무채색. 빨강=a↑ 파랑=b↓ 초록=a↓ 노랑=b↑.
# 기본값은 mock 카메라의 빨간 표식에 맞춰 둠(실물 마커색으로 바꿀 것).
MARKER_LAB = ((40, 150, 0), (255, 255, 255))     # 예: 빨강 밴드
MIN_AREA = 150


def marker_mask(bgr):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    lo, hi = MARKER_LAB
    mask = cv2.inRange(lab, lo, hi)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)


def detect_marker(bgr):
    """가장 큰 마커 → (u, v, angle_deg, contour). 없으면 None.
    angle_deg 는 마커 긴축 방향(놓인 방향 확인용)."""
    cnts, _ = cv2.findContours(marker_mask(bgr), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    (u, v), (w, h), ang = cv2.minAreaRect(c)
    if w < h:                          # 긴축 기준으로 각도 정규화
        ang += 90.0
    return int(u), int(v), float(ang), c


def main():
    H = np.load(H_PATH) if os.path.exists(H_PATH) else None
    if H is None:
        print(f"[경고] {H_PATH} 없음 — 로봇 xy 변환 불가. 03_calib_auto.py 로 먼저 캘리브.")

    win = "iron locate (click=Lab sample, q=quit)"
    cv2.namedWindow(win)
    sample = {"lab": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and param is not None:
            lab = cv2.cvtColor(param, cv2.COLOR_BGR2LAB)[y, x]
            sample["lab"] = (int(lab[0]), int(lab[1]), int(lab[2]))
            print(f"  클릭 Lab @({x},{y}) = L{sample['lab'][0]} a{sample['lab'][1]} b{sample['lab'][2]}")

    with make_camera() as cam:
        last = 0
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            cv2.setMouseCallback(win, on_mouse, rgb)
            det = detect_marker(rgb)
            if det is not None:
                u, v, ang, c = det
                cv2.drawContours(rgb, [c], -1, (0, 255, 0), 2)
                cv2.drawMarker(rgb, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
                txt = f"px({u},{v}) ang{ang:+.0f}"
                if H is not None:
                    x, y = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
                    txt += f"  robot({x:+.3f},{y:+.3f})"
                cv2.putText(rgb, txt, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            else:
                cv2.putText(rgb, "NO marker", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if sample["lab"]:
                cv2.putText(rgb, f"click Lab {sample['lab']}", (8, 48),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.imshow(win, rgb)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
