# -*- coding: utf-8 -*-
"""30_servo.py — 탑다운 비주얼 서보: 인두기 팁을 타겟(드라이버 끝)에 갖다 댄다.

인두기는 집게에 강체로 고정(테이프). 인두기 팁 마커와 타겟(드라이버 끝) 마커를
탑다운으로 함께 보고, H(data/H.npy)로 둘을 '로봇 평면 xy'로 바꿔 오차만큼 팔을
움직여 팁을 타겟 위로 몰아넣는다(폐루프). xy 가 맞으면 부하가 뛸 때까지 하강 =
접촉. 팁 오프셋을 계산할 필요가 없다(브리프 §4.2) — 오차를 눈(카메라)이 없앤다.

폐루프라 부호·게인만 맞으면 수렴한다. H 가 픽셀→로봇을 이미 맞춰 두므로 팔을 로봇
xy 로 +오차만큼 옮기면 팁도 같은 방향으로 간다. 발산하면 GAIN 부호를 뒤집는다.

⚠️ 팔이 자동으로 움직인다. 먼저 MODE="preview"(모션 없음)로 두 마커가 잘 잡히고
   오차 벡터가 맞는지 본 뒤 MODE="servo". 정지는 Enter/Ctrl-C.
키(preview): q=종료
"""
import os
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
_SOARM = os.path.join(_ROOT, "soarm_lab")
for p in (_ROOT, _HERE, _SOARM):
    if p not in sys.path:
        sys.path.insert(0, p)

from hw import make_robot, make_camera
import safety
from ik_core import IKSo101

MODE = "preview"           # "preview"(검출/오차 확인) → "servo"(실제 서보+접촉)

# ── 마커 Lab (lab_tuner.py 로 맞춰 넣기) ──────────────────────────────────────
TIP_LAB = ((40, 150, 0), (255, 255, 255))      # 인두기 팁 마커 (예: 빨강 = a 큼)
TARGET_LAB = ((40, 0, 0), (255, 150, 118))     # 타겟 드라이버 끝 마커 (예: 파랑 = b 작음)
MIN_AREA = 120

# ── 서보 파라미터 ────────────────────────────────────────────────────────────
GAIN = 0.6                 # 오차→이동 비율(<1). 발산하면 부호 뒤집기. 진동하면 낮추기
XY_TOL = 0.004             # 이 거리(m) 안이면 정렬 완료
STEP_CLAMP = 0.012         # 한 번에 움직일 최대 xy(m) — 안전(§10.5)
MAX_ITERS = 40             # 무한 루프 방지(§10.6)
MISS_MAX = 15              # 마커 연속 미검출 허용 횟수

# ── 접근/하강/접촉 ───────────────────────────────────────────────────────────
START_XY = (0.20, 0.0)     # 서보 시작 hover xy(팁이 화면에 보이는 안전한 자리)
Z_HOVER = 0.10             # 정렬 높이(팁이 타겟 위에서 도는 높이)
Z_MIN = 0.01               # 하강 바닥(이보다 밑으론 안 감 — 안전)
Z_STEP = 0.004             # 접촉 탐색 하강 간격(m)

# ── 속도/타이밍 ──────────────────────────────────────────────────────────────
SPEED = 400
ACC = 20
MOVE_SETTLE = 0.7          # 서보는 잦은 소이동이라 짧게
SEED = (0, 30, -45, 0, 0)

H_PATH = os.path.join(_ROOT, "data", "H.npy")
DRAW = {"tip": (0, 0, 255), "target": (255, 0, 0)}
_ik = IKSo101()


# ── 로봇 ─────────────────────────────────────────────────────────────────────
def prep_arm(be):
    drv = be.drv
    if not drv.ping(1):
        raise SystemExit("서보 응답 없음(id1) — 로봇 전원/케이블/포트 확인.")
    for sid in (1, 2, 3, 4, 5):
        drv.set_torque(sid, True)
        drv.set_acceleration(sid, ACC)
        drv.set_speed(sid, SPEED)


def goto(be, est, x, y, z):
    """손끝을 (x,y,z)로(아래보기). 긴급정지·도달·관절한계 검사."""
    est.check()
    angles, err = _ik.solve([float(x), float(y), float(z)],
                            seed_deg=list(SEED), down=True)
    if err > 0.12:
        raise ValueError(f"도달 불가 ({x:.3f},{y:.3f},{z:.3f}) 잔차 {err*1000:.0f}mm")
    bad = safety.check_joint_limits(angles)
    if bad:
        raise safety.Stopped(f"관절 한계 초과로 목표 거부: {bad}")
    be.move(angles, secs=MOVE_SETTLE)
    time.sleep(MOVE_SETTLE)


# ── 검출 ─────────────────────────────────────────────────────────────────────
def detect_pixel(bgr, lab_range):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    lo, hi = lab_range
    mask = cv2.inRange(lab, lo, hi)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    (u, v), _r = cv2.minEnclosingCircle(c)
    return int(u), int(v)


def to_robot(px, H):
    x, y = cv2.perspectiveTransform(np.float32([[[px[0], px[1]]]]), H)[0, 0]
    return float(x), float(y)


def detect_both(bgr, H):
    """(tip_px, tip_xy, tgt_px, tgt_xy). 못 찾은 건 None."""
    tp = detect_pixel(bgr, TIP_LAB)
    gp = detect_pixel(bgr, TARGET_LAB)
    txy = to_robot(tp, H) if (tp and H is not None) else None
    gxy = to_robot(gp, H) if (gp and H is not None) else None
    return tp, txy, gp, gxy


def clamp_vec(dx, dy, maxnorm):
    v = np.array([dx, dy], float)
    n = float(np.linalg.norm(v))
    if n > maxnorm and n > 1e-9:
        v *= maxnorm / n
    return float(v[0]), float(v[1])


def annotate(bgr, tp, gp, err):
    for px, key in ((tp, "tip"), (gp, "target")):
        if px is not None:
            cv2.drawMarker(bgr, px, DRAW[key], cv2.MARKER_CROSS, 18, 2)
            cv2.putText(bgr, key, (px[0] + 6, px[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW[key], 2)
    if tp is not None and gp is not None:
        cv2.arrowedLine(bgr, tp, gp, (0, 255, 255), 2, tipLength=0.2)
    if err is not None:
        cv2.putText(bgr, f"err={err*1000:.0f}mm", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return bgr


# ── 모드 ─────────────────────────────────────────────────────────────────────
def preview(H):
    print("preview: 두 마커 검출/오차 확인 (로봇 안 움직임). q 종료.")
    win = "servo preview (q=quit)"
    with make_camera() as cam:
        last = 0
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            tp, txy, gp, gxy = detect_both(rgb, H)
            err = (np.hypot(gxy[0] - txy[0], gxy[1] - txy[1])
                   if (txy and gxy) else None)
            cv2.imshow(win, annotate(rgb, tp, gp, err))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()


def servo(be, est, H):
    """팁을 타겟 위로 정렬(폐루프) → 부하로 접촉까지 하강 → 후퇴."""
    cur = list(START_XY)
    goto(be, est, cur[0], cur[1], Z_HOVER)          # 시작 hover
    wd = safety.FrameWatchdog()
    last = 0
    miss = 0
    aligned = False
    with make_camera() as cam:
        for it in range(MAX_ITERS):
            est.check()
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None or wd.stalled(last):
                raise safety.Stopped("카메라 프레임 지연/정지")
            tp, txy, gp, gxy = detect_both(rgb, H)
            if txy is None or gxy is None:
                miss += 1
                print(f"  [{it}] 마커 미검출 ({miss}/{MISS_MAX})")
                if miss >= MISS_MAX:
                    raise safety.Stopped("마커를 계속 못 찾음 — 색/조명 확인")
                continue
            miss = 0
            ex, ey = gxy[0] - txy[0], gxy[1] - txy[1]
            err = float(np.hypot(ex, ey))
            print(f"  [{it}] err={err*1000:5.1f}mm  tip({txy[0]:+.3f},{txy[1]:+.3f}) "
                  f"tgt({gxy[0]:+.3f},{gxy[1]:+.3f})")
            if err < XY_TOL:
                aligned = True
                print("  ★ xy 정렬 완료")
                break
            dx, dy = clamp_vec(GAIN * ex, GAIN * ey, STEP_CLAMP)
            cur[0] += dx
            cur[1] += dy
            goto(be, est, cur[0], cur[1], Z_HOVER)
    if not aligned:
        raise safety.Stopped(f"{MAX_ITERS}회 안에 수렴 실패 — GAIN/부호 확인")

    # ── 접촉까지 하강 (부하 감시) ─────────────────────────────────────────────
    guard = safety.LoadGuard(be.drv)
    guard.baseline()
    print("  하강하며 접촉 탐색...", "baseline", guard.base)
    z = Z_HOVER
    while z > Z_MIN:
        est.check()
        if guard.overloaded():
            raise safety.Stopped(f"과부하(≥{guard.abort}) — 즉시정지")
        if guard.contacted():
            print("  ★ 접촉 감지 — 유지 후 후퇴")
            break
        z -= Z_STEP
        goto(be, est, cur[0], cur[1], z)
        print(f"    z={z*1000:.0f}mm  부하상승 {guard.peak_rise():+d}")
    time.sleep(0.6)
    goto(be, est, cur[0], cur[1], Z_HOVER)          # 후퇴
    print("완료.")


def main():
    H = np.load(H_PATH) if os.path.exists(H_PATH) else None
    if MODE == "preview":
        if H is None:
            print(f"[경고] {H_PATH} 없음 — 로봇 xy·오차 표시 불가(픽셀만).")
        preview(H)
        return
    if MODE == "servo":
        if H is None:
            raise SystemExit(f"{H_PATH} 없음 — 03_calib_auto.py 로 먼저 캘리브.")
        be = make_robot()
        prep_arm(be)
        est = safety.EStop(on_stop=lambda: be.drv.set_all_torque(False))
        print("[안전] 작업면에서 손을 치우세요. 정지는 Enter/Ctrl-C.")
        input("서보 시작하려면 Enter > ")
        est.start()                                 # 확인 뒤 watcher 가동
        try:
            servo(be, est, H)
        except safety.Stopped as e:
            print(f"\n중단: {e}")
            be.drv.set_all_torque(False)
        except KeyboardInterrupt:
            est.trip("Ctrl-C")
        return
    raise SystemExit(f"알 수 없는 MODE: {MODE!r} (preview 또는 servo)")


if __name__ == "__main__":
    main()
