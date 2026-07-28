# -*- coding: utf-8 -*-
"""06_pick.py — 빨간 공 하나를 집어서 지정 위치에 내려놓기 (실물 pick & place).

검출(빨강, Lab) → H로 로봇 xy → 접근·하강·집기·들기·이동·놓기. 04_click_move 에서
검증된 arm.go(real=True) 이동에 실물 그리퍼(real.RealBackend.grip)를 얹은 것.
먼저 03_calib_auto.py 로 data/H.npy 를 만들어야 한다.

⚠️ 로봇이 자동으로 공을 향해 움직이고 집는다. 작업면에 손을 넣지 말 것.
    MODE="grip_test" 로 그리퍼 여닫힘 값(OPEN/CLOSE_FRAC)부터 맞춘 뒤 MODE="pick".

두 모드:
  grip_test : 콘솔에 0(닫힘)~1(열림) 입력 → 그리퍼만 여닫으며 CLOSE_FRAC 찾기. q=종료.
  pick      : 빨간 공 1개를 집어 DROP_XY 로 옮기고 내려놓는다.
"""
import os
import time

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab import arm

MODE = "pick"          # "grip_test" 로 그리퍼값부터 맞춘 뒤 "pick"
REAL = True

# ── 그리퍼(fraction: 0=닫힘 .. 1=열림) ─────────────────────────────────────────
OPEN_FRAC = 1.0
CLOSE_FRAC = 0.3           # 공을 쥘 만큼만 닫음(grip_test 로 실측해 조정)

# ── 높이(로봇 z, m) ─────────────────────────────────────────────────────────
Z_HOVER = 0.12              # 접근/이동 시 손끝 높이(공 위)
Z_GRASP = 0.03              # 집을 때 손끝 높이(테이블 근처 — 실물서 튜닝)
Z_DROP = 0.04              # 놓을 때 손끝 높이

# ── 파지 기하 ────────────────────────────────────────────────────────────────
# 순서: 공 뒤(베이스쪽)에서 hover 접근 → 앞으로(바깥으로) 이동 → 하강 → 집기.
# radial(+)=베이스에서 바깥(앞) 방향. '덜 가서 집음' → GRASP_FWD 를 키운다.
GRASP_FWD = 0.02             # 공 대비 앞으로 보정(m). 덜 가면 +로 키움(예: 0.01~0.03)
APPROACH_BACK = 0.05        # 하강 전 공 뒤(베이스쪽)에서 이만큼 떨어져 접근(m)
DROP_XY = (0.16, -0.12)     # 내려놓을 위치(로봇 xy) — 단일 공 테스트용 고정값

# ── 속도/타이밍 ──────────────────────────────────────────────────────────────
SPEED = 500                 # 팔 서보 속도(작을수록 느림)
ACC = 20                    # 팔 가감속
MOVE_SETTLE = 1.3           # 각 이동 후 대기(실물이 목표에 도달할 시간)
GRIP_SETTLE = 0.7           # 그리퍼 여닫은 뒤 대기

# ── 빨간 공 검출(Lab: 빨강 = a 큼) ───────────────────────────────────────────
L_MIN = 40
RED_A_MIN = 150             # a 하한(빨강일수록 큼). 파랑도 잡으려면 별도 b 범위로.
MIN_AREA = 150


def prep_arm():
    """팔 서보 토크 켜고 속도/가속 낮춤(03 이 꺼둔 토크 복구 · 홱 방지)."""
    drv = arm._backend(True).drv
    if not drv.ping(1):
        raise SystemExit("서보 응답 없음(id1) — 로봇 전원/케이블/포트 확인 후 다시 실행.")
    for sid in (1, 2, 3, 4, 5):
        drv.set_torque(sid, True)
        drv.set_acceleration(sid, ACC)
        drv.set_speed(sid, SPEED)


def grip(frac):
    """실물 그리퍼 여닫기(0=닫힘..1=열림) + 정착 대기."""
    arm._backend(True).grip(frac)
    time.sleep(GRIP_SETTLE)


def goto(x, y, z):
    """손끝을 (x,y,z)로 이동(아래보기) + 정착 대기."""
    arm.go([float(x), float(y), float(z)], real=REAL, down=True)
    time.sleep(MOVE_SETTLE)


def detect_red(bgr):
    """빨간 공 → (u,v,r). Lab 의 a 채널(빨강=큼)로 검출. 못 찾으면 None."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(lab, (L_MIN, RED_A_MIN, 0), (255, 255, 255))
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


def find_ball_xy(H):
    """카메라로 빨간 공을 안정 검출해 로봇 xy 반환. 창에서 확인 후 Space=확정 / q=취소."""
    win = "pick: red ball (Space=confirm / q=cancel)"
    cv2.namedWindow(win)
    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            det = detect_red(rgb)
            if det is not None:
                u, v, r = det
                cv2.circle(rgb, (u, v), r, (0, 0, 255), 2)
                cv2.drawMarker(rgb, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
                cv2.putText(rgb, "Space=confirm", (u - 45, v - r - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            else:
                cv2.putText(rgb, "NO red ball", (8, 24),
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


def radial(xy, d):
    """xy를 베이스→점 방향(바깥=+)으로 d(m) 옮긴다. d>0 앞으로, d<0 뒤로."""
    x, y = xy
    th = np.arctan2(y, x)
    return x + d * np.cos(th), y + d * np.sin(th)


def pick_place(obj_xy):
    """공 뒤에서 접근 → 앞으로 이동 → 하강 → 집기 → 들기 → DROP_XY 로 이동 → 놓기."""
    gx, gy = radial(obj_xy, GRASP_FWD)                 # 파지 xy(공 ± 앞뒤 보정)
    hx, hy = radial((gx, gy), -APPROACH_BACK)          # 접근 xy(파지점 뒤=베이스쪽)
    px, py = DROP_XY

    grip(OPEN_FRAC)                       # 열고 시작
    goto(hx, hy, Z_HOVER)                 # ① 공 뒤 위로 접근
    goto(gx, gy, Z_HOVER)                 # ② 앞으로 이동(파지점 위)
    goto(gx, gy, Z_GRASP)                 # ③ 집게 내리기(하강)
    grip(CLOSE_FRAC)                      # ④ 닫기
    goto(gx, gy, Z_HOVER)                 # ⑤ 들기
    goto(px, py, Z_HOVER)                 # ⑥ 놓을 곳 위로
    goto(px, py, Z_DROP)                  # ⑦ 내리기
    grip(OPEN_FRAC)                       # ⑧ 놓기
    goto(px, py, Z_HOVER)                 # ⑨ 물러나기
    print("완료.")


def grip_test():
    """그리퍼 여닫힘 값 찾기: 0(닫힘)~1(열림) 입력 → 그때그때 여닫음. q=종료."""
    print("그리퍼 테스트: 0(닫힘)~1(열림) 입력. 공을 쥐는 최소 닫힘값을 CLOSE_FRAC 로. q=종료.")
    while True:
        s = input("frac 0~1 (q=종료) > ").strip()
        if s == "q":
            break
        try:
            f = float(s)
        except ValueError:
            print("  0~1 사이 숫자 또는 q")
            continue
        arm._backend(True).grip(f)
        print(f"  → grip {f:.2f}")


def main():
    if not REAL:
        raise SystemExit("이 스크립트는 실물 전용입니다. REAL=True 로 두고 실행하세요.")
    prep_arm()

    if MODE == "grip_test":
        grip_test()
        return

    if MODE == "pick":
        H = np.load(os.path.join("data", "H.npy"))
        print(f"[안전] 작업면에 빨간 공 1개만 두고 손을 치우세요. (CLOSE_FRAC={CLOSE_FRAC})")
        obj = find_ball_xy(H)
        if obj is None:
            print("취소.")
            return
        print(f"공 로봇좌표 ({obj[0]:+.3f}, {obj[1]:+.3f}) → 집어서 {DROP_XY} 로 이동")
        input("실행하려면 Enter (중단 Ctrl-C) > ")
        pick_place(obj)
        return

    raise SystemExit(f"알 수 없는 MODE: {MODE!r} (grip_test 또는 pick)")


if __name__ == "__main__":
    main()
