# -*- coding: utf-8 -*-
"""calibrate.py — 4점 호모그래피로 '픽셀 → 로봇 좌표' 변환을 만든다.

작업면 네 곳에 빨간 공을 옮겨 놓으면서, 매번 (카메라가 본 픽셀, 로봇이 있는 xy)
한 쌍을 모은다. 네 쌍이면 평면끼리의 사영변환 H 가 결정된다. 이후 파이프라인은
H 곱셈 한 번으로 화면 좌표를 로봇 좌표로 바꾼다.

로봇 xy 를 얻는 방법: 토크를 끄고 손으로 팔끝을 공에 댄 뒤 관절각을 읽어 FK 로
푼다(티칭). 로봇 자신이 "내 손끝이 지금 여기"라고 알려주는 값이라 자尺가 필요없다.

실행:
    python -m sorting.calibrate

⚠ 포트를 단독으로 쓴다 — 컨트롤러 앱이나 main.py 는 미리 종료할 것.
"""
from __future__ import annotations

import os

import cv2
import numpy as np
from hp60c_camera import CameraReader

from soarm_lab.driver_sdk import STS3215Driver
from soarm_lab.fk_core import FKSo101

from . import config as cfg
from .vision import detect_balls

N_POINTS = 4
GOOD_ERROR_M = 0.01     # 재투영 오차가 이보다 크면 4점이 너무 몰려 있다는 뜻


def _largest_red(bgr):
    """가장 큰 빨간 공 하나. 캘리브레이션 때는 공을 하나만 두므로 이걸로 충분하다."""
    balls = detect_balls(bgr, cfg.RED)
    return max(balls, key=lambda b: b.area) if balls else None


def _hud(img, text, color):
    for c, tk in (((0, 0, 0), 3), (color, 1)):
        cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, tk,
                    cv2.LINE_AA)


def grab_pixel(cam) -> tuple[int, int]:
    """빨간 공을 자동 검출해 보여주고 Space 로 확정. 검출이 안 되면 클릭. q=취소."""
    win = "calibrate: Space=확정 / 클릭=수동 / q=취소"
    clicked: dict[str, tuple[int, int]] = {}

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked["uv"] = (x, y)

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    last = 0
    try:
        while True:
            rgb, _depth, last = cam.read_blocking(last)
            if rgb is None:
                continue
            view = rgb.copy()
            ball = _largest_red(view)
            if ball is not None:
                cv2.circle(view, ball.uv, ball.r, cfg.RED.draw, 2)
                cv2.drawMarker(view, ball.uv, (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
                _hud(view, f"공 검출 {ball.area:.0f}px — Space 로 확정", (0, 255, 0))
            else:
                # 왜 Space 가 안 먹는지 화면에 말해준다 — 조용한 실패를 없앤다
                _hud(view, "공이 안 보임 — 클릭으로 지정하거나 lab_sample 로 색 재측정",
                     (0, 200, 255))
            cv2.imshow(win, view)

            key = cv2.waitKey(1) & 0xFF
            if "uv" in clicked:
                return clicked["uv"]
            if key == ord(" ") and ball is not None:
                return ball.uv
            if key == ord("q"):
                raise KeyboardInterrupt
    finally:
        cv2.destroyWindow(win)


def read_robot_xy(drv: STS3215Driver, fk: FKSo101) -> tuple[float, float]:
    """토크를 끄고 팔끝을 공에 댄 뒤 Enter — 그때 관절각을 FK 로 풀어 xy 를 얻는다."""
    for sid in (1, 2, 3, 4, 5):
        drv.set_torque(sid, False)
    input("  팔끝을 공에 대고 Enter > ")
    pos = drv.get_all_positions()
    if any(pos.get(i + 1) is None for i in range(5)):
        raise SystemExit(
            "서보 응답 없음 — 전원/케이블/포트를 확인하세요.\n"
            "(이대로 진행하면 네 점이 모두 같은 좌표로 읽혀 H 가 엉터리가 됩니다)")
    deg = [STS3215Driver.position_to_degrees(pos.get(i + 1)) or 0.0 for i in range(5)]
    p, _ = fk.fk_deg(deg)
    return float(p[0]), float(p[1])


def reprojection_error(H, pixels, robots) -> float:
    """모은 네 점을 H 로 다시 투영해 원래 로봇좌표와 얼마나 어긋나는지(m)."""
    worst = 0.0
    for (u, v), (x, y) in zip(pixels, robots):
        px, py = cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]
        worst = max(worst, float(np.hypot(px - x, py - y)))
    return worst


def main() -> None:
    drv = STS3215Driver(port=cfg.ROBOT.port)
    drv.connect()
    fk = FKSo101()
    pixels: list[tuple[int, int]] = []
    robots: list[tuple[float, float]] = []

    print("4점 캘리브레이션 — 공을 작업면에 '넓게' 흩어 놓을수록 정확해집니다.")
    try:
        with CameraReader() as cam:
            for i in range(N_POINTS):
                print(f"[{i + 1}/{N_POINTS}] 공을 새 위치에 놓고 Space(또는 클릭).")
                uv = grab_pixel(cam)
                xy = read_robot_xy(drv, fk)
                print(f"  픽셀 {uv} → 로봇 ({xy[0]:+.3f}, {xy[1]:+.3f})")
                pixels.append(uv)
                robots.append(xy)
    except KeyboardInterrupt:
        print("취소 — 저장하지 않았습니다.")
        return
    finally:
        cv2.destroyAllWindows()

    H = cv2.getPerspectiveTransform(np.float32(pixels), np.float32(robots))
    err = reprojection_error(H, pixels, robots)
    verdict = "양호" if err < GOOD_ERROR_M else "큼 — 네 점을 더 넓게 잡아 다시 하세요"
    print(f"재투영 최대오차 {err * 1000:.1f}mm ({verdict})")

    os.makedirs(cfg.DATA, exist_ok=True)
    np.save(cfg.H_FILE, H)
    print("저장:", cfg.H_FILE)


if __name__ == "__main__":
    main()
