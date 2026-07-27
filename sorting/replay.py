# -*- coding: utf-8 -*-
"""replay.py — 저장된 사진을 카메라인 척 흘려주는 소스.

카메라도 로봇도 없이 GUI·검출·상태머신을 통째로 돌려보기 위한 것이다.
테스트가 이걸 물려 쓰기 때문에 진입점 스크립트가 아니라 패키지 안에 둔다.

    from sorting.replay import ReplaySource
    with ReplaySource("shots") as cam:
        rgb, depth, fid = cam.read_blocking(0)
"""
from __future__ import annotations

import glob
import os
import time

import cv2


class ReplaySource:
    """hp60c_camera.CameraReader 와 같은 모양으로 저장된 사진을 돌려준다.

    read_blocking 의 시그니처와 반환 형태(rgb, depth, frame_id)를 그대로
    맞춰서, 위쪽 코드는 진짜 카메라인지 아닌지 모른 채로 동작한다.

    depth 는 None 이다 — 사진에는 깊이가 없으므로 안전감시는 유휴 상태가 된다.
    """

    def __init__(self, pattern: str, fps: float = 10.0):
        paths = sorted(glob.glob(os.path.join(pattern, "*.png"))
                       if os.path.isdir(pattern) else glob.glob(pattern))
        if not paths:
            raise SystemExit(f"재생할 사진이 없습니다: {pattern}")
        self.frames = [f for f in (cv2.imread(p) for p in paths) if f is not None]
        if not self.frames:
            raise SystemExit(f"사진을 읽지 못했습니다: {pattern}")
        self.period = 1.0 / fps
        self._i = 0
        print(f"[replay] 사진 {len(self.frames)}장을 카메라 대신 사용합니다")

    def read_blocking(self, last_frame_id: int = 0, timeout: float = 1.0,
                      poll_hz: float = 200.0):
        time.sleep(self.period)
        frame = self.frames[self._i % len(self.frames)]
        self._i += 1
        return frame.copy(), None, last_frame_id + 1

    def __enter__(self) -> "ReplaySource":
        return self

    def __exit__(self, *exc) -> bool:
        return False
