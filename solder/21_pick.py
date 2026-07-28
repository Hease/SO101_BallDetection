# -*- coding: utf-8 -*-
"""21_pick.py — 탑다운으로 검출한 인두기를 고정 방향으로 파지한다 (실물 pick).

'카메라 위치검출(방향 고정)' 픽업. 20_locate 로 맞춘 마커색으로 인두기 위치를 잡아
H(data/H.npy)로 로봇 xy 를 얻고, 어제 06_pick 에서 검증된 접근·하강·집기·들기 모션을
그대로 쓴다. 인두기는 일정 방향으로 놓인다고 가정하므로 접근 방위(APPROACH_AZIM_DEG)는
고정한다(None 이면 베이스→물체 방향으로 접근).

어제 06_pick 과 다른 점(요청 반영):
  · 모션을 hw.make_robot() 백엔드로 보낸다 → 실물/mock 자동, 포트 자동 해석
  · 안전 골격 적용: 긴급정지(Enter/Ctrl-C), 관절한계 초과 목표 거부, 저속

⚠️ 로봇이 자동으로 인두기를 향해 움직이고 집는다. 작업면에 손을 넣지 말 것.
   MODE="grip_test" 로 인두기 쥐는 CLOSE_FRAC 부터 맞춘 뒤 MODE="pick".
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

MODE = "pick"              # "grip_test"(집게값 맞추기) → "pick"

# ── 그리퍼(0=닫힘 .. 1=열림) ─────────────────────────────────────────────────
OPEN_FRAC = 1.0
CLOSE_FRAC = 0.3           # 인두기 배럴을 쥐는 값 — grip_test 로 실측
# ── 높이(로봇 z, m) ─────────────────────────────────────────────────────────
Z_HOVER = 0.12
Z_GRASP = 0.03
# ── 파지 기하 ────────────────────────────────────────────────────────────────
GRASP_FWD = 0.02           # 파지점 앞뒤 보정(m). 덜 가면 +로 키움
APPROACH_BACK = 0.05       # 하강 전 파지점 뒤에서 접근하는 거리(m)
APPROACH_AZIM_DEG = None   # 접근 방위(도). None=베이스→물체 방향. 인두기 방향 고정이면 숫자로.
DROP_XY = (0.16, -0.12)    # 임시 놓을 위치(이송→접촉은 다음 단계에서 붙임)
# ── 속도/타이밍 ──────────────────────────────────────────────────────────────
SPEED = 400
ACC = 20
MOVE_SETTLE = 1.3
GRIP_SETTLE = 0.7
# ── 인두기 마커(Lab) — 20_locate 와 같은 값으로 맞출 것 ──────────────────────
MARKER_LAB = ((40, 150, 0), (255, 255, 255))
MIN_AREA = 150

SEED = (0, 30, -45, 0, 0)  # IK 시작 추정치(도)
H_PATH = os.path.join(_ROOT, "data", "H.npy")

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


def grip(be, frac):
    be.grip(frac)
    time.sleep(GRIP_SETTLE)


def goto(be, est, x, y, z):
    """손끝을 (x,y,z)로 이동(아래보기). 긴급정지·관절한계 검사 후 명령."""
    est.check()                                     # 긴급정지
    angles, err = _ik.solve([float(x), float(y), float(z)],
                            seed_deg=list(SEED), down=True)
    if err > 0.12:                                  # 아래보기 잔차 허용
        raise ValueError(f"도달 불가 ({x:.3f},{y:.3f},{z:.3f}) 잔차 {err*1000:.0f}mm")
    bad = safety.check_joint_limits(angles)
    if bad:                                         # 관절한계 초과 목표 거부(§10.5)
        raise safety.Stopped(f"관절 한계 초과로 목표 거부: {bad}")
    be.move(angles, secs=MOVE_SETTLE)
    time.sleep(MOVE_SETTLE)


# ── 검출 ─────────────────────────────────────────────────────────────────────
def detect_marker(bgr):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    lo, hi = MARKER_LAB
    mask = cv2.inRange(lab, lo, hi)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    (u, v), r = cv2.minEnclosingCircle(c)
    return int(u), int(v), int(r)


def find_marker_xy(cam, H):
    """인두기 마커를 안정 검출해 로봇 xy 반환. Space=확정 / q=취소."""
    win = "pick: iron marker (Space=confirm / q=cancel)"
    cv2.namedWindow(win)
    last = 0
    while True:
        rgb, _d, last = cam.read_blocking(last)
        if rgb is None:
            continue
        det = detect_marker(rgb)
        if det is not None:
            u, v, r = det
            cv2.circle(rgb, (u, v), r, (0, 255, 0), 2)
            cv2.drawMarker(rgb, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
            cv2.putText(rgb, "Space=confirm", (u - 45, v - r - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        else:
            cv2.putText(rgb, "NO marker", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow(win, rgb)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(" ") and det is not None:
            cv2.destroyWindow(win)
            x, y = cv2.perspectiveTransform(np.float32([[[det[0], det[1]]]]), H)[0, 0]
            return float(x), float(y)
        if k == ord("q"):
            cv2.destroyWindow(win)
            return None


def offset(xy, d, azim_deg):
    """xy 를 접근 방위로 d(m) 옮긴다. azim_deg=None 이면 베이스→물체 방향."""
    x, y = xy
    th = np.radians(azim_deg) if azim_deg is not None else np.arctan2(y, x)
    return x + d * np.cos(th), y + d * np.sin(th)


def pick_place(be, est, obj_xy):
    """접근 → 하강 → 집기 → 들기 → DROP_XY 로 이동 → 놓기. 매 이동 안전검사."""
    gx, gy = offset(obj_xy, GRASP_FWD, APPROACH_AZIM_DEG)         # 파지 xy
    hx, hy = offset((gx, gy), -APPROACH_BACK, APPROACH_AZIM_DEG)  # 접근 xy(파지점 뒤)
    px, py = DROP_XY
    grip(be, OPEN_FRAC)
    goto(be, est, hx, hy, Z_HOVER)      # ① 인두기 뒤 위로 접근
    goto(be, est, gx, gy, Z_HOVER)      # ② 앞으로
    goto(be, est, gx, gy, Z_GRASP)      # ③ 하강
    grip(be, CLOSE_FRAC)                # ④ 집기
    goto(be, est, gx, gy, Z_HOVER)      # ⑤ 들기
    goto(be, est, px, py, Z_HOVER)      # ⑥ 놓을 곳 위로
    goto(be, est, px, py, Z_GRASP + 0.02)  # ⑦ 내리기
    grip(be, OPEN_FRAC)                 # ⑧ 놓기
    goto(be, est, px, py, Z_HOVER)      # ⑨ 물러나기
    print("완료.")


def grip_test(be):
    print("그리퍼 테스트: 0(닫힘)~1(열림) 입력 → 여닫음. 인두기 쥐는 값을 CLOSE_FRAC 로. q=종료.")
    while True:
        s = input("frac 0~1 (q=종료) > ").strip()
        if s == "q":
            break
        try:
            be.grip(float(s))
        except ValueError:
            print("  0~1 숫자 또는 q")


def main():
    be = make_robot()                              # 실물 or mock(print), 포트 자동
    prep_arm(be)
    est = safety.EStop(on_stop=lambda: be.drv.set_all_torque(False))

    if MODE == "grip_test":
        grip_test(be)
        return
    if MODE == "pick":
        if not os.path.exists(H_PATH):
            raise SystemExit(f"{H_PATH} 없음 — 03_calib_auto.py 로 먼저 캘리브.")
        H = np.load(H_PATH)
        print(f"[안전] 인두기 1개만 두고 손을 치우세요. (CLOSE_FRAC={CLOSE_FRAC})")
        with make_camera() as cam:
            obj = find_marker_xy(cam, H)
        if obj is None:
            print("취소.")
            return
        print(f"인두기 로봇좌표 ({obj[0]:+.3f}, {obj[1]:+.3f}) → 집어서 {DROP_XY} 로")
        input("실행하려면 Enter (정지 Enter/Ctrl-C) > ")
        est.start()                                # 확인 뒤 긴급정지 watcher 가동
        try:
            pick_place(be, est, obj)
        except safety.Stopped as e:
            print(f"\n중단: {e}")
            be.drv.set_all_torque(False)
        except KeyboardInterrupt:
            est.trip("Ctrl-C")
        return
    raise SystemExit(f"알 수 없는 MODE: {MODE!r} (grip_test 또는 pick)")


if __name__ == "__main__":
    main()
