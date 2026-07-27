# -*- coding: utf-8 -*-
"""07_zones.py — 탑다운 영상 위에 색깔별 '내려놓을 영역(존)'을 사각형으로 지정.

마우스 드래그로 사각형을 그리면 현재 색(빨강/파랑)의 존이 된다. 존은 픽셀 좌표로
data/zones.json 에 저장되고, 08_sort.py 가 읽어 존 중심을 H로 로봇좌표로 바꿔 공을
그리로 옮긴다. (로봇은 안 움직인다 — 카메라만 필요.)

키: r=빨강 존 · b=파랑 존 · (드래그=그리기) · u=마지막 취소 · c=전체 지움
    s=저장 · q=종료
"""
import json
import os

import cv2
from hp60c_camera import CameraReader

OUT = os.path.join("data", "zones.json")
COLORS = {"red": (0, 0, 255), "blue": (255, 0, 0)}      # BGR 표시색


def load_zones():
    if os.path.exists(OUT):
        with open(OUT) as f:
            return json.load(f)
    return {}


def main():
    zones = load_zones()                # {"red":[x0,y0,x1,y1], "blue":[...]}
    state = {"color": "red", "drag": None, "cur": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["drag"] = (x, y)
            state["cur"] = (x, y, x, y)
        elif event == cv2.EVENT_MOUSEMOVE and state["drag"] is not None:
            x0, y0 = state["drag"]
            state["cur"] = (x0, y0, x, y)
        elif event == cv2.EVENT_LBUTTONUP and state["drag"] is not None:
            x0, y0 = state["drag"]
            rect = [min(x0, x), min(y0, y), max(x0, x), max(y0, y)]
            if rect[2] - rect[0] > 5 and rect[3] - rect[1] > 5:     # 너무 작은 건 무시
                zones[state["color"]] = rect
            state["drag"] = None
            state["cur"] = None

    win = "zones (r=red b=blue / drag / u=undo c=clear s=save q=quit)"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    print("드래그로 존 그리기. r/b=색 선택 · u=취소 · c=지움 · s=저장 · q=종료")

    with CameraReader() as cam:
        last = 0
        while True:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            for color, rect in zones.items():               # 저장된 존
                x0, y0, x1, y1 = rect
                bgr = COLORS.get(color, (0, 255, 255))
                cv2.rectangle(rgb, (x0, y0), (x1, y1), bgr, 2)
                cv2.putText(rgb, color, (x0 + 4, y0 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)
            if state["cur"] is not None:                     # 그리는 중
                x0, y0, x1, y1 = state["cur"]
                cv2.rectangle(rgb, (x0, y0), (x1, y1),
                              COLORS[state["color"]], 1)
            cv2.putText(rgb, f"active: {state['color']}", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLORS[state["color"]], 2)
            cv2.imshow(win, rgb)

            k = cv2.waitKey(1) & 0xFF
            if k == ord("r"):
                state["color"] = "red"
            elif k == ord("b"):
                state["color"] = "blue"
            elif k == ord("u"):
                zones.pop(state["color"], None)              # 현재 색 존 취소
            elif k == ord("c"):
                zones.clear()
            elif k == ord("s"):
                os.makedirs("data", exist_ok=True)
                with open(OUT, "w") as f:
                    json.dump(zones, f, indent=2)
                print("저장:", OUT, zones)
            elif k == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
