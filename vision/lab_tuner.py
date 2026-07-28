# -*- coding: utf-8 -*-
"""lab_tuner.py — Lab 트랙바로 색 범위(숫자 6개)를 뽑는 도구. hsv_tuner 의 Lab 판.

슬라이더로 L/a/b 의 최소·최대를 움직이며 마스크(흰=선택된 색)를 실시간으로 본다.
대상만 하얗게 남는 값을 찾으면 그 6개 숫자가 결과물 → 02_detect · 20_locate ·
08_sort 등의 cv2.inRange(lab, lo, hi) 임계값에 그대로 넣는다.

Lab 는 조도(밝기 L)와 색(a·b)이 분리돼 HSV 보다 조명 변화에 강하다.
OpenCV 8bit Lab: L,a,b ∈ [0,255], a·b 는 128 이 무채색.
  빨강 = a 큼(>128) · 초록 = a 작음 · 노랑 = b 큼(>128) · 파랑 = b 작음.
→ 보통 L 은 넓게(40~255) 두고 a 또는 b 한두 축만 좁혀 색을 가른다. HSV 같은
  색상환 양끝 처리(Hmin>Hmax)가 필요 없다.

카메라:        python lab_tuner.py
저장 사진으로: python lab_tuner.py shots/shot_000.png
키: p=현재 값 출력 · q=종료   |  마우스 클릭=그 점의 Lab 값 출력(색 맞추기용)
"""
import sys

import cv2

WIN = "lab tuner (click=sample, p=print, q=quit)"
BARS = [("Lmin", 40, 255), ("Lmax", 255, 255),
        ("amin", 128, 255), ("amax", 255, 255),
        ("bmin", 0, 255), ("bmax", 255, 255)]


def _noop(_):
    pass


def make_mask(bgr):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    p = {n: cv2.getTrackbarPos(n, WIN) for n, _, _ in BARS}
    lo = (p["Lmin"], p["amin"], p["bmin"])
    hi = (p["Lmax"], p["amax"], p["bmax"])
    return cv2.inRange(lab, lo, hi), lo, hi


def main():
    src_img = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else None
    cv2.namedWindow(WIN)
    for name, val, mx in BARS:
        cv2.createTrackbar(name, WIN, val, mx, _noop)

    sample = {"lab": None, "frame": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and sample["frame"] is not None:
            lab = cv2.cvtColor(sample["frame"], cv2.COLOR_BGR2LAB)[y, x]
            sample["lab"] = (int(lab[0]), int(lab[1]), int(lab[2]))
            print(f"클릭 Lab @({x},{y}) = L{sample['lab'][0]} a{sample['lab'][1]} b{sample['lab'][2]}")

    cv2.setMouseCallback(WIN, on_mouse)

    cam = None
    if src_img is None:
        from hp60c_camera import CameraReader
        cam = CameraReader()
    last = 0
    print("슬라이더로 대상만 하얗게 남기세요. 클릭=Lab 샘플, p=값 출력, q=종료")
    while True:
        if src_img is not None:
            bgr = src_img.copy()
        else:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            bgr = rgb
        sample["frame"] = bgr                        # 클릭 콜백이 볼 현재 프레임
        mask, lo, hi = make_mask(bgr)
        masked = cv2.bitwise_and(bgr, bgr, mask=mask)
        cv2.putText(masked, f"LO={lo}  HI={hi}", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if sample["lab"]:
            cv2.putText(masked, f"click Lab={sample['lab']}", (8, 48),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        cv2.imshow(WIN, cv2.hconcat([bgr, masked]))
        k = cv2.waitKey(1) & 0xFF
        if k == ord("p"):
            print(f"LO={lo}  HI={hi}")
        elif k == ord("q"):
            break
    if cam is not None:
        cam.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
