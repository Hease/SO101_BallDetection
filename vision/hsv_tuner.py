# -*- coding: utf-8 -*-
"""hsv_tuner.py — Lab 트랙바로 색 기준값(a, b, radius)을 뽑는 도구 (정답지).

HSV 대신 CIELAB 을 쓰는 이유: L(밝기)과 a*/b*(색기)가 분리돼 있어 그늘이 져도
색 판정이 거의 안 흔들리고, 빨강도 색상환 이음매(0·180)가 없어 두 구간으로
쪼개 OR 할 필요가 없다. 판정은 딱 하나 — "a*/b* 평면에서 기준색까지의 거리가
radius 이내인가" + "L이 Lmin~Lmax 범위인가".

색을 화면에서 **클릭**하면 그 픽셀의 (a, b)를 기준색 트랙바에 바로 반영한다.
그 다음 radius/Lmin/Lmax 슬라이더로 마스크가 원하는 물체만 하얗게 남을 때까지
좁히면 끝 — HSV 6슬라이더보다 훨씬 빠르다.

카메라:        python hsv_tuner.py
저장 사진으로: python hsv_tuner.py shots/shot_000.png
키: 클릭=기준색(a,b) 샘플 · p=현재 값 출력 · q=종료
"""
import sys

import cv2
import numpy as np

WIN = "lab tuner (click=sample, p=print, q=quit)"
BARS = [("a+128", 128, 255), ("b+128", 128, 255),
        ("radius", 20, 100),
        ("Lmin", 25, 255), ("Lmax", 245, 255)]


def _noop(_):
    pass


def lab_channels(bgr):
    """BGR → (L, a*, b*). a*/b* 는 OpenCV 의 128 오프셋을 뺀 실제 값(float32)."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    L, a, b = cv2.split(lab)
    return L.astype(np.float32), a.astype(np.float32) - 128.0, b.astype(np.float32) - 128.0


def make_mask(bgr):
    L, a_chan, b_chan = lab_channels(bgr)
    p = {n: cv2.getTrackbarPos(n, WIN) for n, _, _ in BARS}
    a_ref, b_ref = p["a+128"] - 128, p["b+128"] - 128
    radius = p["radius"]
    l_min, l_max = p["Lmin"], p["Lmax"]

    da, db = a_chan - a_ref, b_chan - b_ref
    within = (da * da + db * db <= radius * radius) & (L >= l_min) & (L <= l_max)
    mask = within.astype(np.uint8) * 255
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)       # 잔점 제거
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern)      # 구멍 메우기
    return mask, (a_ref, b_ref, radius), (l_min, l_max)


def main():
    src_img = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else None
    cv2.namedWindow(WIN)
    for name, val, mx in BARS:
        cv2.createTrackbar(name, WIN, val, mx, _noop)

    clicked = {}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked["uv"] = (x, y)

    cv2.setMouseCallback(WIN, on_mouse)

    cam = None
    if src_img is None:
        from hp60c_camera import CameraReader
        cam = CameraReader()
    last = 0
    print("물체를 클릭해 기준색을 잡고, radius/Lmin/Lmax로 다듬으세요. p=값 출력, q=종료")
    while True:
        if src_img is not None:
            bgr = src_img.copy()
        else:
            rgb, _d, last = cam.read_blocking(last)
            if rgb is None:
                continue
            bgr = rgb

        if "uv" in clicked:                        # 클릭 픽셀의 (a,b)를 트랙바에 반영
            x, y = clicked.pop("uv")
            if 0 <= y < bgr.shape[0] and 0 <= x < bgr.shape[1]:
                _L, a_chan, b_chan = lab_channels(bgr)
                cv2.setTrackbarPos("a+128", WIN, int(a_chan[y, x]) + 128)
                cv2.setTrackbarPos("b+128", WIN, int(b_chan[y, x]) + 128)

        mask, (a_ref, b_ref, radius), (l_min, l_max) = make_mask(bgr)
        masked = cv2.bitwise_and(bgr, bgr, mask=mask)
        cv2.putText(masked, f"a={a_ref:+.0f} b={b_ref:+.0f} r={radius} L=[{l_min},{l_max}]",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.imshow(WIN, cv2.hconcat([bgr, masked]))
        k = cv2.waitKey(1) & 0xFF
        if k == ord("p"):
            print(f"a={a_ref:+.0f}  b={b_ref:+.0f}  radius={radius}  L=({l_min},{l_max})")
        elif k == ord("q"):
            break
    if cam is not None:
        cam.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
