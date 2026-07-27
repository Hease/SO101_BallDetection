# -*- coding: utf-8 -*-
"""08_sort.py — 공 색 분류: 빨강/파랑 공을 각자 색 존으로 옮긴다 (실물).

07_zones.py 로 만든 data/zones.json(색별 픽셀 사각형)과 03_calib_auto.py 의
data/H.npy 를 읽는다. 매 스캔마다 모든 공을 검출해 '자기 색 존 밖'에 있는 공을
하나 집어 그 색 존으로 옮기고, 다시 스캔한다. 모든 공이 제 존에 들면 종료.

검출은 Lab(빨강=a 큼 · 파랑=b 작음)로 조도에 강하게. 파지 시퀀스는 06_pick 과 동일
(공 뒤 접근 → 앞으로 → 하강 → 집기). 06_pick 에서 맞춘 값을 아래에 반영해 둔다.

⚠️ 로봇이 자동으로 공을 집어 옮긴다. 작업면에 손을 넣지 말 것.
    먼저 MODE="preview"(로봇 안 움직임)로 검출/존이 맞는지 확인 후 MODE="sort".
키(preview): q=종료
"""
import json
import os
import time

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab import arm

MODE = "preview"            # "preview"(검출/존 확인) → "sort"(실제 분류)
REAL = True

# ── 그리퍼(0=닫힘 .. 1=열림) · 높이 · 파지기하 (06_pick 튜닝값) ─────────────────
OPEN_FRAC = 1.0
CLOSE_FRAC = 0.3
Z_HOVER = 0.12
Z_GRASP = 0.03
Z_DROP = 0.04
GRASP_FWD = 0.02            # 공 대비 앞으로 보정(m)
APPROACH_BACK = 0.05        # 하강 전 공 뒤에서 떨어지는 거리(m)

# ── 스캔 중 팔을 치워둘 파크 자세(공을 가리지 않게) ──────────────────────────
PARK_XYZ = (0.14, 0.0, 0.20)

# ── 속도/타이밍 ──────────────────────────────────────────────────────────────
SPEED = 500
ACC = 20
MOVE_SETTLE = 1.3
GRIP_SETTLE = 0.7

# ── 공 검출(Lab) ─────────────────────────────────────────────────────────────
L_MIN = 40
MIN_AREA = 150
# 색별 Lab 범위 (lo, hi) = ((L,a,b), (L,a,b)). OpenCV 8bit: a,b 는 128 이 무채색.
COLOR_LAB = {
    "red":  ((L_MIN, 150, 0),   (255, 255, 255)),   # 빨강 = a 큼
    "blue": ((L_MIN, 0, 0),     (255, 150, 118)),   # 파랑 = b 작음
}
DRAW = {"red": (0, 0, 255), "blue": (255, 0, 0)}

H_PATH = os.path.join("data", "H.npy")
ZONES_PATH = os.path.join("data", "zones.json")


# ── 검출 ─────────────────────────────────────────────────────────────────────
def color_mask(bgr, color):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    lo, hi = COLOR_LAB[color]
    mask = cv2.inRange(lab, lo, hi)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)


def detect_balls(bgr):
    """모든 빨강/파랑 공 → [{'color','u','v','r'}, ...] (면적 큰 순)."""
    out = []
    for color in COLOR_LAB:
        cnts, _ = cv2.findContours(color_mask(bgr, color),
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) < MIN_AREA:
                continue
            (u, v), r = cv2.minEnclosingCircle(c)
            out.append({"color": color, "u": int(u), "v": int(v), "r": int(r)})
    return sorted(out, key=lambda d: d["r"], reverse=True)


def grab(cam, n=6):
    """정착 위해 몇 프레임 흘려보내고 마지막 유효 프레임 반환."""
    frame, last = None, 0
    for _ in range(n):
        rgb, _d, last = cam.read_blocking(0 if frame is None else last)
        if rgb is not None:
            frame = rgb
    return frame


def in_rect(u, v, rect):
    x0, y0, x1, y1 = rect
    return x0 <= u <= x1 and y0 <= v <= y1


def annotate(bgr, dets, zones):
    for color, rect in zones.items():
        x0, y0, x1, y1 = rect
        cv2.rectangle(bgr, (x0, y0), (x1, y1), DRAW.get(color, (0, 255, 255)), 2)
        cv2.putText(bgr, color, (x0 + 4, y0 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, DRAW.get(color, (0, 255, 255)), 2)
    for d in dets:
        done = d["color"] in zones and in_rect(d["u"], d["v"], zones[d["color"]])
        col = (0, 200, 0) if done else DRAW[d["color"]]
        cv2.circle(bgr, (d["u"], d["v"]), d["r"], col, 2)
        cv2.putText(bgr, d["color"] + ("*" if done else ""), (d["u"] - d["r"], d["v"] - d["r"] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    return bgr


# ── 실물 파지 (06_pick 과 동일) ──────────────────────────────────────────────
def prep_arm():
    drv = arm._backend(True).drv
    if not drv.ping(1):
        raise SystemExit("서보 응답 없음(id1) — 로봇 전원/케이블/포트 확인 후 다시 실행.")
    for sid in (1, 2, 3, 4, 5):
        drv.set_torque(sid, True)
        drv.set_acceleration(sid, ACC)
        drv.set_speed(sid, SPEED)


def grip(frac):
    arm._backend(True).grip(frac)
    time.sleep(GRIP_SETTLE)


def goto(x, y, z):
    arm.go([float(x), float(y), float(z)], real=REAL, down=True)
    time.sleep(MOVE_SETTLE)


def radial(xy, d):
    x, y = xy
    th = np.arctan2(y, x)
    return x + d * np.cos(th), y + d * np.sin(th)


def pick_place(obj_xy, drop_xy):
    gx, gy = radial(obj_xy, GRASP_FWD)
    hx, hy = radial((gx, gy), -APPROACH_BACK)
    px, py = drop_xy
    grip(OPEN_FRAC)
    goto(hx, hy, Z_HOVER)                 # 공 뒤 위로 접근
    goto(gx, gy, Z_HOVER)                 # 앞으로
    goto(gx, gy, Z_GRASP)                 # 하강
    grip(CLOSE_FRAC)                      # 집기
    goto(gx, gy, Z_HOVER)                 # 들기
    goto(px, py, Z_HOVER)                 # 존 위로
    goto(px, py, Z_DROP)                  # 내리기
    grip(OPEN_FRAC)                       # 놓기
    goto(px, py, Z_HOVER)                 # 물러나기


def park():
    goto(*PARK_XYZ)                       # 팔을 치워 카메라가 작업면을 보게


def zone_drop_xy(rect, H, idx):
    """존 사각형 안에서 idx 번째 놓을 지점(픽셀 3x3 격자)을 로봇 xy로 변환.
    여러 공을 같은 존에 쌓지 않도록 조금씩 흩어 놓는다."""
    x0, y0, x1, y1 = rect
    gx = [0.25, 0.5, 0.75]
    cx = x0 + (x1 - x0) * gx[idx % 3]
    cy = y0 + (y1 - y0) * gx[(idx // 3) % 3]
    x, y = cv2.perspectiveTransform(np.float32([[[cx, cy]]]), H)[0, 0]
    return float(x), float(y)


# ── 모드 ─────────────────────────────────────────────────────────────────────
def preview(zones):
    print("preview: 검출/존 확인 (로봇 안 움직임). *=이미 제 존 안. q 종료.")
    win = "sort preview (q=quit)"
    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            dets = detect_balls(rgb)
            cv2.imshow(win, annotate(rgb, dets, zones))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


def sort_loop(zones, H):
    prep_arm()
    print("[안전] 작업면에서 손을 치우세요. 색 존 밖의 공을 하나씩 옮깁니다.")
    input("실행하려면 Enter (중단 Ctrl-C) > ")
    placed = {c: 0 for c in zones}
    with CameraReader() as cam:
        while True:
            park()                                    # 팔 치우고
            frame = grab(cam)                         # 스캔
            if frame is None:
                continue
            dets = detect_balls(frame)
            todo = [d for d in dets
                    if d["color"] in zones and not in_rect(d["u"], d["v"], zones[d["color"]])]
            skipped = [d["color"] for d in dets if d["color"] not in zones]
            if skipped:
                print("존 없는 색 무시:", set(skipped))
            if not todo:
                print("완료 — 모든 공이 제 색 존 안에 있습니다.")
                break
            d = todo[0]
            obj_xy = cv2.perspectiveTransform(
                np.float32([[[d["u"], d["v"]]]]), H)[0, 0]
            drop_xy = zone_drop_xy(zones[d["color"]], H, placed[d["color"]])
            print(f"{d['color']} 공 픽셀({d['u']},{d['v']}) "
                  f"-> 집어서 {d['color']} 존으로 (남은 {len(todo)})")
            pick_place((float(obj_xy[0]), float(obj_xy[1])), drop_xy)
            placed[d["color"]] += 1
    park()


def main():
    if not os.path.exists(ZONES_PATH):
        raise SystemExit(f"{ZONES_PATH} 없음 — 먼저 07_zones.py 로 존을 지정하세요.")
    with open(ZONES_PATH) as f:
        zones = json.load(f)
    if not zones:
        raise SystemExit("존이 비었습니다 — 07_zones.py 에서 s 로 저장했는지 확인.")

    if MODE == "preview":
        preview(zones)
        return
    if MODE == "sort":
        if not REAL:
            raise SystemExit("sort 는 실물 전용입니다. REAL=True.")
        H = np.load(H_PATH)
        sort_loop(zones, H)
        return
    raise SystemExit(f"알 수 없는 MODE: {MODE!r} (preview 또는 sort)")


if __name__ == "__main__":
    main()
