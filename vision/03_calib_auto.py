# -*- coding: utf-8 -*-
"""03_calib_auto.py — 자동 그리드 캘리브레이션 (픽셀 <-> 로봇, 강건판).

기존 03_map.py 의 문제(4점만 정확맞춤 · 공 윗면 픽셀 vs 손끝 접촉점 어긋남 ·
FK 원시값 unwrap 누락으로 인한 좌우 비대칭)를 우회한다.

원리
----
그리퍼 끝에 눈에 띄는 마커(기본: 초록)를 붙이고, 로봇을 '아는 좌표(x,y)'들의
격자로 스스로 이동(항상 아래보기 down=True)시킨다. 각 지점에서 마커 픽셀을 자동
검출해 (픽셀 <-> 명령좌표)를 모은다. 나중에 공을 검출해 같은 arm.go 로 보내므로
"이 픽셀 = 집게 끝을 여기로 보내는 명령"이 그대로 성립한다(자기일관적).
FK 를 전혀 읽지 않으므로 서보 원시값/홈오프셋 버그와 무관하다.

- 다점(격자) + cv2.findHomography(RANSAC) → 한 점 튀어도 걸러내고 최소자승으로 안정.
- 저장 전 화면에 격자를 되투영해 눈으로 검증(s=저장 / q=취소).
- 도달불가 지점은 콘솔에 표시 → 좌우 비대칭이 매핑 문제인지 도달범위 문제인지 진단.

먼저 시뮬로 격자 도달성을 확인한 뒤(REAL=False) 실물(REAL=True)로.
⚠️ 실물은 로봇이 격자를 따라 자동으로 움직인다. 작업면을 비우고 손을 넣지 말 것.
키(수집 중): 마커 자동검출되면 자동확정 · 클릭=수동확정 · n=이 점 건너뜀 · q=중단
키(검증 중): s=저장 · q=취소
"""
import os
import time

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab import arm

# ── 마커 HSV (그리퍼 끝에 붙인 색) ──────────────────────────────────────────
# 기본은 초록(빨강/파랑 공과 겹치지 않게). hsv_tuner.py 로 "마커만 하얗게" 남긴
# 값을 넣는다. 빨강 마커면 Hmin>Hmax(예: (160,120,80)/(10,255,255))로 적으면
# 아래 detect_marker 가 색상환 양끝을 감싼다(OR).
MARKER_LO = (40, 80, 60)
MARKER_HI = (85, 255, 255)
MIN_AREA = 150

# ── 격자(로봇 좌표계, m) ────────────────────────────────────────────────────
# 관측된 작업범위 근처의 보수적 기본값. 시뮬 프리뷰에서 도달불가가 많으면 좁힌다.
X_RANGE = (0.16, 0.28)      # 베이스에서 바깥 방향
Y_RANGE = (-0.12, 0.12)     # 좌우
NX, NY = 4, 4               # 격자 해상도(4x4=16점)
Z_CAL = 0.10                # 캘리브 시 손끝 높이(m) — 테이블 위 안전한 hover
                            # (탑다운이라 xy 매핑은 높이와 무관 → 파지 Z는 따로 정함)

# ── 실물 동작 ───────────────────────────────────────────────────────────────
REAL = False                # 먼저 False(시뮬 프리뷰)로 격자 확인 → True 로 실측
SPEED = 500                 # 실물 서보 속도(작을수록 느림 · 안전)
ACC = 20                    # 실물 가감속(작을수록 부드럽게)
SETTLE_SEC = 1.2            # 이동 후 정지 대기(마커가 멈춘 뒤 검출)
DETECT_FRAMES = 8           # 검출 안정화를 위해 모을 프레임 수(중앙값 사용)

OUT = os.path.join("data", "H.npy")


def detect_marker(bgr):
    """마커 → (u, v, r). 못 찾으면 None. (02_detect 와 같은 HSV 로직, 빨강 wrap 지원)"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lo, hi = MARKER_LO, MARKER_HI
    if lo[0] <= hi[0]:
        mask = cv2.inRange(hsv, lo, hi)
    else:                                   # 색상환 양끝 감싸기(빨강 마커용)
        mask = (cv2.inRange(hsv, (lo[0], lo[1], lo[2]), (179, hi[1], hi[2])) |
                cv2.inRange(hsv, (0, lo[1], lo[2]), (hi[0], hi[1], hi[2])))
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


def grid_points():
    """작업면을 덮는 로봇 (x,y) 격자. 뱀(boustrophedon) 순서로 이동거리 최소화."""
    xs = np.linspace(X_RANGE[0], X_RANGE[1], NX)
    ys = np.linspace(Y_RANGE[0], Y_RANGE[1], NY)
    pts = []
    for i, x in enumerate(xs):
        row = ys if i % 2 == 0 else ys[::-1]    # 지그재그
        for y in row:
            pts.append((float(x), float(y)))
    return pts


def prep_real():
    """실물 서보 토크 켜고 속도/가속 낮춤(03_map 이 꺼둔 토크 복구 · 홱 방지)."""
    drv = arm._backend(True).drv
    if not drv.ping(1):
        raise SystemExit("서보 응답 없음(id1) — 로봇 전원/케이블/포트 확인 후 다시 실행.")
    for sid in (1, 2, 3, 4, 5):
        drv.set_torque(sid, True)
        drv.set_acceleration(sid, ACC)
        drv.set_speed(sid, SPEED)


def median_detect(cam):
    """정지 대기 후 여러 프레임에서 마커를 검출해 중앙값 픽셀 반환. (프레임, 픽셀|None)."""
    t_end = time.monotonic() + SETTLE_SEC
    last = 0
    frame = None
    while time.monotonic() < t_end:            # 이동 잔떨림 가라앉히기
        rgb, _d, last = cam.read_blocking(last)
        if rgb is not None:
            frame = rgb
    us, vs = [], []
    got = 0
    while got < DETECT_FRAMES:
        rgb, _d, last = cam.read_blocking(last)
        if rgb is None:
            continue
        frame = rgb
        det = detect_marker(rgb)
        if det is not None:
            us.append(det[0]); vs.append(det[1])
        got += 1
    if len(us) >= DETECT_FRAMES // 2:          # 절반 이상 검출돼야 신뢰
        return frame, (int(np.median(us)), int(np.median(vs)))
    return frame, None


def collect_real():
    """실물: 격자를 자동 순회하며 (픽셀, 로봇좌표) 수집. 반환 (pixels, robots, unreachable)."""
    prep_real()
    pts = grid_points()
    pixels, robots, unreachable = [], [], []
    win = "calib (auto=confirm / click=manual / n=skip / q=abort)"
    cv2.namedWindow(win)
    click = {}
    cv2.setMouseCallback(win, lambda e, x, y, f, p:
                         click.update(uv=(x, y)) if e == cv2.EVENT_LBUTTONDOWN else None)

    print(f"[안전] 작업면을 비우세요. 로봇이 {len(pts)}개 격자점을 자동 이동합니다.")
    input("준비되면 Enter (중단은 이후 창에서 q) > ")

    with CameraReader() as cam:
        for i, (x, y) in enumerate(pts):
            try:
                arm.go([x, y, Z_CAL], real=True, down=True)   # 아래보기로 마커 일관 배치
            except ValueError as e:
                print(f"[{i+1}/{len(pts)}] ({x:+.3f},{y:+.3f}) 도달불가 — 건너뜀: {e}")
                unreachable.append((x, y))
                continue

            click.pop("uv", None)
            frame, uv = median_detect(cam)
            # 검출/수동확정 루프 — 자동검출 성공이면 바로 확정, 아니면 클릭/스킵 대기
            while True:
                disp = frame.copy() if frame is not None else np.zeros((480, 640, 3), np.uint8)
                if uv is not None:
                    cv2.drawMarker(disp, uv, (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
                msg = f"[{i+1}/{len(pts)}] robot({x:+.3f},{y:+.3f})  " + \
                      ("AUTO ok" if uv else "NO marker: click / n=skip")
                cv2.putText(disp, msg, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(disp, msg, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.imshow(win, disp)
                k = cv2.waitKey(30) & 0xFF
                if "uv" in click:                       # 수동확정
                    uv = click.pop("uv"); break
                if uv is not None:                      # 자동확정
                    break
                if k == ord("n"):                       # 이 점 건너뜀
                    uv = None; break
                if k == ord("q"):
                    raise KeyboardInterrupt
                _f, uv = median_detect(cam)             # 다시 검출 시도

            if uv is not None:
                print(f"[{i+1}/{len(pts)}] 픽셀{uv} <- 로봇({x:+.3f},{y:+.3f})")
                pixels.append(uv); robots.append((x, y))
            else:
                print(f"[{i+1}/{len(pts)}] 마커 미검출 — 건너뜀")
    cv2.destroyWindow(win)
    return pixels, robots, unreachable


def preview_sim():
    """시뮬 프리뷰: 카메라 없이 격자를 순회하며 도달성만 확인(실물 전 점검)."""
    pts = grid_points()
    ok = 0
    print(f"시뮬 프리뷰 — 격자 {len(pts)}점 도달성 확인")
    for i, (x, y) in enumerate(pts):
        try:
            _a, err = arm.go([x, y, Z_CAL], real=False, down=True, secs=0.4)
            print(f"[{i+1}/{len(pts)}] ({x:+.3f},{y:+.3f}) OK  잔차 {err*1000:.0f}mm")
            ok += 1
        except ValueError as e:
            print(f"[{i+1}/{len(pts)}] ({x:+.3f},{y:+.3f}) 도달불가: {e}")
    print(f"\n도달 {ok}/{len(pts)}. 도달불가가 많으면 X_RANGE/Y_RANGE 를 좁히세요.")
    print("도달성이 만족스러우면 파일 상단 REAL=True 로 바꿔 실물 캘리브를 실행하세요.")
    arm.wait(real=False)


def fit_and_report(pixels, robots):
    """findHomography(RANSAC) 로 H 추정 + 점별 재투영오차(mm) 보고. 반환 (H, worst_mm)."""
    src = np.float32(pixels).reshape(-1, 1, 2)
    dst = np.float32(robots).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, ransacReprojThreshold=0.006)
    if H is None:
        raise SystemExit("호모그래피 추정 실패 — 점이 부족하거나 일직선입니다. 격자를 넓히세요.")
    proj = cv2.perspectiveTransform(src, H).reshape(-1, 2)
    errs = np.linalg.norm(proj - np.float32(robots), axis=1) * 1000.0   # mm
    print("\n점별 재투영오차:")
    for (u, v), (x, y), e, m in zip(pixels, robots, errs, mask.ravel()):
        tag = "" if m else "  <-- RANSAC 이상치(제외됨)"
        print(f"  픽셀({u:4d},{v:4d}) -> ({x:+.3f},{y:+.3f})  오차 {e:5.1f}mm{tag}")
    inl = errs[mask.ravel() == 1]
    print(f"\ninlier {int(mask.sum())}/{len(pixels)} · RMS {np.sqrt((inl**2).mean()):.1f}mm"
          f" · 최대 {inl.max():.1f}mm",
          "(양호)" if inl.max() < 8 else "(큼 — 격자를 더 넓게/촘촘히, 마커 검출 확인)")
    return H, float(inl.max())


def verify(H):
    """저장 전 검증: 로봇 격자를 H⁻¹로 픽셀에 되투영해 겹쳐 본다. s=저장 / q=취소."""
    Hinv = np.linalg.inv(H)
    grid = np.float32(grid_points()).reshape(-1, 1, 2)
    px = cv2.perspectiveTransform(grid, Hinv).reshape(-1, 2)
    win = "verify: green=grid reprojection | s=save q=cancel"
    cv2.namedWindow(win)
    clicked = {}
    cv2.setMouseCallback(win, lambda e, x, y, f, p:
                         clicked.update(uv=(x, y)) if e == cv2.EVENT_LBUTTONDOWN else None)
    print("검증: 초록점이 실제 격자 위치와 맞는지 확인. 클릭하면 예측 로봇좌표 출력. s=저장 q=취소.")
    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            for (u, v) in px:
                cv2.circle(rgb, (int(u), int(v)), 4, (0, 255, 0), -1)
            if "uv" in clicked:
                u, v = clicked["uv"]
                x, y = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
                print(f"  클릭({u},{v}) -> 예측 로봇({x:+.3f},{y:+.3f})")
                clicked.pop("uv")
            cv2.imshow(win, rgb)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("s"):
                cv2.destroyWindow(win)
                return True
            if k == ord("q"):
                cv2.destroyWindow(win)
                return False


def main():
    if not REAL:
        preview_sim()
        return

    try:
        pixels, robots, unreachable = collect_real()
    except KeyboardInterrupt:
        print("중단 — 저장 안 함")
        return
    finally:
        cv2.destroyAllWindows()

    if unreachable:
        print(f"\n도달불가 {len(unreachable)}점:", unreachable)
        print("→ 한쪽만 도달불가면 '매핑'이 아니라 로봇 '도달범위' 문제입니다(X/Y_RANGE 조정).")
    if len(pixels) < 4:
        raise SystemExit(f"수집 {len(pixels)}점 < 4 — 캘리브 불가. 마커 색/격자범위를 확인하세요.")

    H, worst = fit_and_report(pixels, robots)

    if verify(H):
        os.makedirs("data", exist_ok=True)
        np.save(OUT, H)
        print("저장:", OUT)
    else:
        print("취소 — 저장 안 함")


if __name__ == "__main__":
    main()
