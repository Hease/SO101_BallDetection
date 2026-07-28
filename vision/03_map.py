# -*- coding: utf-8 -*-
"""03_map.py — N점 호모그래피(least-squares): 픽셀 <-> 로봇 좌표.

작업면 여러 곳(N곳, 기본 7)에 빨간 마커를 두고, 각 지점에서 카메라가 자동검출한
마커 픽셀과 로봇 xy 를 짝지어 모은다. 픽셀은 빨간 마커를 Lab로 자동검출해 화면에
표시하고 Space 로 확정한다(검출이 안 되면 마우스 클릭으로 대체, q=취소). 로봇 xy 는
토크를 끄고 팔끝을 마커에 댄 뒤 관절각을 FK 로 읽어 얻는다. 결과 H 는 data/H.npy 로
저장되어 04_click_move / 05_track 이 불러 쓴다.

정확히 4점으로 cv2.getPerspectiveTransform 을 쓰면 그 4점을 항상 오차 0으로 맞추는
exact-fit 이라, 점 하나의 측정 실수(서보 응답 지연·접촉 오차 등)가 곧바로 H 전체를
왜곡시키고 진단할 방법도 없다. N>4 로 여유 있게 모아 cv2.findHomography(RANSAC) 로
풀면 이상치 하나가 전체를 끌고 가지 않고, 점별 오차도 볼 수 있어 어느 점이 잘못됐는지
알 수 있다.
포트 /dev/ttyACM1 단독(컨트롤러 앱·다른 프로세스는 종료).
"""

import os

import cv2
import numpy as np
from hp60c_camera import CameraReader
from soarm_lab.driver_sdk import STS3215Driver
from soarm_lab.fk_core import FKSo101

N = 7  # 4보다 넉넉히 — 이상치에 흔들리지 않으려면 여유(redundancy)가 필요
PORT = "/dev/ttyACM1"
OUT = os.path.join("data", "H.npy")

# 드라이버 빨강 마킹 Lab 검출용(02_detect 와 동일 임계값; 자체완결 위해 인라인)
# 캘리브레이션은 점 하나를 픽셀↔로봇좌표로 잇는 기하학적 절차라 색 자체는 의미가
# 없다 — 검출이 잘 되는 색 하나(빨강)만 있으면 충분하고, 여기서 얻은 H 는 색과 무관하게
# 테이블 평면 위 어떤 픽셀이든 그대로 변환해준다(파랑 손잡이 등도 이 H 를 그대로 재사용).
# hsv_tuner.py 로 마커를 클릭해 (a, b, radius)를 뽑아 옮겨 적는다.
RED = (56, 22, 43)       # (a, b, radius) — 실측값 (02_detect.py 와 동일)
L_MIN, L_MAX = 25, 245   # 그림자/반사 컷 — 색 구분은 a*/b* 가 하므로 넉넉하게 둔다
MIN_AREA = 150


def detect_red(bgr):
    """빨간 마커(드라이버 전체를 감싼 경우 포함) → ((p1,p2,r), 최대덩어리 면적).
    못 찾으면 (None, 면적). 면적을 같이 주는 이유: 못 찾았을 때 '왜'를 화면에 띄우기 위해서.
    판정은 하나뿐: a*/b* 평면에서 기준색까지의 거리가 radius 이내인가.

    드라이버 전체가 빨강이면 덩어리가 길쭉해서 중심(minEnclosingCircle)은 몸통
    한가운데를 가리킨다 — 어느 쪽이 끝단인지는 색만으로 구분이 안 되므로, 장축
    양 끝점 p1/p2 를 둘 다 계산해서 반환하고 최종 선택은 호출부(마우스 위치)에 맡긴다."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    L, a_chan, b_chan = cv2.split(lab)
    L = L.astype(np.float32)
    a_chan = a_chan.astype(np.float32) - 128.0
    b_chan = b_chan.astype(np.float32) - 128.0
    a_ref, b_ref, radius = RED
    da, db = a_chan - a_ref, b_chan - b_ref
    within = (da * da + db * db <= radius * radius) & (L >= L_MIN) & (L <= L_MAX)
    mask = within.astype(np.uint8) * 255
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)       # 잔점 제거
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern)      # 구멍 메우기
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, 0.0
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA:
        return None, area
    (cx, cy), (w, h), ang = cv2.minAreaRect(c)
    th = np.deg2rad(ang if w >= h else ang + 90)      # 긴 변 방향
    axis = np.array([np.cos(th), np.sin(th)])
    half_len = max(w, h) / 2
    center = np.array([cx, cy])
    p1, p2 = center + axis * half_len, center - axis * half_len
    r = max(2, int(min(w, h) / 2))                    # 시각화용(짧은 변 절반)
    return (p1, p2, r), area


def get_pixel(cam):
    """빨간 마커를 자동검출해 표시하고 Space 로 그 위치 확정. 검출 없으면 클릭. q=취소.

    마커(드라이버)가 길쭉하면 끝점 후보가 둘(p1/p2) 나오는데, 색만으로는 어느 쪽이
    끝단인지 알 수 없으므로 **마우스를 가까이 댄 쪽**을 자동 선택으로 삼는다 —
    원하는 끝 근처로 커서를 옮긴 뒤 Space. 여전히 클릭하면 그 정확한 픽셀을 쓴다."""
    win = "map: hover=pick tip / Space=confirm / click=manual / q=cancel"
    clicked = {}
    mouse_xy = [0, 0]

    def on_mouse(event, x, y, flags, param):
        mouse_xy[0], mouse_xy[1] = x, y
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked["uv"] = (x, y)

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    last = 0
    try:
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            det, area = detect_red(rgb)
            auto_uv = None
            if det is not None:
                p1, p2, r = det
                mx, my = mouse_xy
                d1 = (p1[0] - mx) ** 2 + (p1[1] - my) ** 2
                d2 = (p2[0] - mx) ** 2 + (p2[1] - my) ** 2
                tip = p1 if d1 <= d2 else p2       # 마우스에 더 가까운 끝점 = 추적 대상
                auto_uv = (int(tip[0]), int(tip[1]))
                for p in (p1, p2):                 # 후보 끝점 둘 다 표시(선택 안 된 쪽은 작게)
                    cv2.circle(rgb, (int(p[0]), int(p[1])), 4, (60, 60, 255), -1)
                cv2.circle(rgb, auto_uv, r, (0, 0, 255), 2)
                cv2.drawMarker(rgb, auto_uv, (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
                cv2.putText(
                    rgb,
                    "Space=confirm (mouse selects nearer tip)",
                    (auto_uv[0] - 45, auto_uv[1] - r - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    2,
                )
            else:  # 왜 못 찾았는지 표시 — 조용히 Space 가 안 먹는 상황 방지
                msg = (
                    f"NO MARKER: max blob {area:.0f}px < MIN_AREA {MIN_AREA}"
                    if area
                    else "NO MARKER: red pixels 0"
                )
                for c_, tk in (((0, 0, 0), 3), ((0, 200, 255), 1)):
                    cv2.putText(
                        rgb, msg, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c_, tk
                    )
            cv2.imshow(win, rgb)
            k = cv2.waitKey(1) & 0xFF
            if "uv" in clicked:  # 수동: 클릭한 픽셀
                return clicked["uv"]
            if k == ord(" ") and auto_uv is not None:  # 자동: 마우스가 고른 끝점 확정
                return auto_uv
            if k == ord("q"):
                raise KeyboardInterrupt
    finally:
        cv2.destroyWindow(win)


def robot_xy(drv, fk):
    for sid in (1, 2, 3, 4, 5):
        drv.set_torque(sid, False)
    input("  팔끝을 마커에 대고 Enter > ")
    pos = drv.get_all_positions()
    if any(pos.get(i + 1) is None for i in range(5)):
        raise SystemExit(
            "서보 응답 없음 — 로봇 전원/케이블 확인. "
            "(이대로 계속하면 모든 점이 같은 좌표로 읽혀 H가 엉터리가 된다)"
        )
    deg = [STS3215Driver.position_to_degrees(pos.get(i + 1)) or 0.0 for i in range(5)]
    p, _ = fk.fk_deg(deg)
    return float(p[0]), float(p[1])


def main():
    if RED == (0, 0, 0):
        raise SystemExit(
            "먼저: hsv_tuner.py 로 구한 값을 파일 상단 RED 에 채우세요."
        )

    drv = STS3215Driver(port=PORT)
    drv.connect()
    fk = FKSo101()
    pixels, robots = [], []
    try:
        with CameraReader() as cam:
            for i in range(N):
                print(
                    f"[{i + 1}/{N}] 마커를 새 위치에 놓고(작업면 구석구석 넓게), Space로 확정(또는 클릭)."
                )
                uv = get_pixel(cam)
                xy = robot_xy(drv, fk)
                print(f"  픽셀 {uv} -> 로봇 ({xy[0]:+.3f}, {xy[1]:+.3f})")
                pixels.append(uv)
                robots.append(xy)
    except KeyboardInterrupt:
        print("취소 — 저장하지 않음")
        return
    finally:
        cv2.destroyAllWindows()

    # reprojThreshold 는 dst(로봇, m) 기준 — 5.0 같은 픽셀 스케일 값을 그대로 쓰면
    # RANSAC이 사실상 전부 inlier로 보고 이상치를 못 걸러낸다. 1cm로 잡는다.
    H, inliers = cv2.findHomography(
        np.float32(pixels), np.float32(robots), cv2.RANSAC, 0.01
    )

    worst = 0.0
    for i, ((u, v), (x, y)) in enumerate(zip(pixels, robots)):
        px, py = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
        err = ((px - x) ** 2 + (py - y) ** 2) ** 0.5
        worst = max(worst, err)
        flag = "" if inliers[i, 0] else "  ← 이상치(다시 잴 것)"
        print(f"  점 {i + 1}: 오차 {err * 1000:5.1f}mm{flag}")
    print(
        f"재투영 최대오차 {worst * 1000:.1f}mm",
        "(양호)" if worst < 0.01 else "(큼 — 위에서 오차 큰 점만 골라 다시 재보정)",
    )

    os.makedirs("data", exist_ok=True)
    np.save(OUT, H)
    print("저장:", OUT)


if __name__ == "__main__":
    main()
