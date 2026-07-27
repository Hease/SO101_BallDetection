# -*- coding: utf-8 -*-
"""mapping.py — 화면 픽셀 ↔ 로봇 좌표.

calibrate.py 가 만든 3x3 호모그래피 H 를 읽어 쓴다. 카메라와 작업면이 각각
평면이라 둘 사이는 사영변환 하나로 정확히 이어진다 — 그래서 깊이 정보 없이도
'화면의 이 점'이 '책상 위 그 점'인지 알 수 있다.

주의: H 는 **책상면 위의 점**에만 맞는다. 공중에 뜬 물체는 원근 때문에 어긋난다.
"""
from __future__ import annotations

import os

import cv2
import numpy as np

from . import config as cfg


class Mapper:
    """픽셀 (u,v) → 로봇 (x,y) 변환기."""

    def __init__(self, H: np.ndarray):
        self.H = np.asarray(H, dtype=np.float64)
        self._Hinv = np.linalg.inv(self.H)

    @classmethod
    def load(cls, path: str | None = None) -> "Mapper":
        path = path or cfg.H_FILE
        if not os.path.exists(path):
            raise SystemExit(
                f"캘리브레이션 파일이 없습니다: {path}\n"
                "먼저 실행하세요:  python -m sorting.calibrate")
        return cls(np.load(path))

    def to_robot(self, u: float, v: float) -> tuple[float, float]:
        """픽셀 한 점 → 로봇 (x, y) [m]."""
        pt = np.float32([[[float(u), float(v)]]])
        x, y = cv2.perspectiveTransform(pt, self.H)[0, 0]
        return float(x), float(y)

    def to_pixel(self, x: float, y: float) -> tuple[float, float]:
        """로봇 (x, y) → 픽셀. 안전 경계선을 로봇 좌표로 그릴 때 쓴다."""
        pt = np.float32([[[float(x), float(y)]]])
        u, v = cv2.perspectiveTransform(pt, self._Hinv)[0, 0]
        return float(u), float(v)
