# -*- coding: utf-8 -*-
"""safety.py — 작업면에 사람 손이 들어오면 알아챈다 (깊이 기반 안전 경계선).

왜 '경계선' 방식인가:

    로봇팔도 책상 위에 높게 떠 있다. 그래서 "책상보다 높은 것"을 전부 침입으로
    보면 로봇 자신이 계속 걸린다. 대신 **로봇이 절대 넘지 않는 바깥 띠**만
    감시한다. 팔은 띠 안쪽에서만 움직이고, 사람 손은 반드시 띠를 통과해
    들어오므로, 손이 팔에 닿기 전에 잡힌다. 산업용 안전펜스와 같은 원리다.

색이 아니라 깊이를 쓰는 이유: 피부색·장갑·소매 색을 가리지 않고, 조명이
바뀌어도 흔들리지 않는다. HP60C 가 이미 깊이를 주므로 추가 장비도 없다.

    python -m sorting.safety          # 로봇 없이 감지만 눈으로 확인
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass

import cv2
import numpy as np

from . import config as cfg


@dataclass
class Zone:
    """로봇 작업영역 폴리곤. 좌표는 0~1 정규화 — 컬러/깊이 해상도가 달라도 통한다.

    HP60C 는 RGB 와 깊이의 해상도·렌즈가 서로 다르다(브리지가 IR/RGB intrinsic 을
    따로 찍는다). 픽셀 좌표를 그대로 저장하면 한쪽에서만 맞는다.
    """
    polygon_norm: list[tuple[float, float]]

    def to_pixels(self, shape: tuple[int, int]) -> np.ndarray:
        h, w = shape[:2]
        return np.array([[int(x * w), int(y * h)] for x, y in self.polygon_norm],
                        dtype=np.int32)

    def band_mask(self, shape: tuple[int, int]) -> np.ndarray:
        """감시할 띠 = 화면 전체에서 로봇 작업영역을 뺀 나머지."""
        h, w = shape[:2]
        inside = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(inside, [self.to_pixels(shape)], 255)
        return cv2.bitwise_not(inside)

    def save(self, path: str | None = None) -> str:
        path = path or cfg.ZONE_FILE
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump({"polygon_norm": [list(p) for p in self.polygon_norm]}, fh,
                      indent=2)
        return path

    @classmethod
    def load(cls, path: str | None = None) -> "Zone | None":
        path = path or cfg.ZONE_FILE
        if not os.path.exists(path):
            return None
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
        return cls([tuple(p) for p in data["polygon_norm"]])


class SafetyMonitor:
    """깊이 프레임을 계속 먹여주면 침입 여부를 알려준다.

    zone 이나 baseline 이 없으면 **항상 '침입 없음'** 을 돌려준다. 안전장치가
    없는데 있는 척하지 않기 위해, 그 경우 enabled=False 로 표시해 GUI 가
    "안전감시 꺼짐"을 눈에 띄게 알릴 수 있게 한다.
    """

    def __init__(self, zone: Zone | None = None, baseline: np.ndarray | None = None,
                 conf: cfg.SafetyConfig | None = None):
        self.conf = conf or cfg.SAFETY
        self.zone = zone
        self.baseline = baseline
        self._mask: np.ndarray | None = None
        self._mask_shape: tuple[int, int] | None = None
        self._threshold = self.conf.min_pixels_floor

        self.intruded = False
        self._hot = 0        # 연속 '침입' 프레임
        self._cold = 0       # 연속 '깨끗' 프레임
        self.last_pixels = 0

    @classmethod
    def load(cls, conf: cfg.SafetyConfig | None = None) -> "SafetyMonitor":
        zone = Zone.load()
        baseline = np.load(cfg.BASELINE_FILE) if os.path.exists(cfg.BASELINE_FILE) else None
        return cls(zone, baseline, conf)

    @property
    def enabled(self) -> bool:
        return self.zone is not None and self.baseline is not None

    def _band_for(self, shape) -> np.ndarray:
        if self._mask is None or self._mask_shape != shape[:2]:
            self._mask = self.zone.band_mask(shape)
            self._mask_shape = shape[:2]
            band_px = int(np.count_nonzero(self._mask))
            self._threshold = max(self.conf.min_pixels_floor,
                                  int(band_px * self.conf.min_band_fraction))
        return self._mask

    @property
    def threshold_px(self) -> int:
        """이번 해상도에서 침입으로 볼 픽셀 수. 띠 면적에 비례해 정해진다."""
        return self._threshold

    def _raw_intrusion(self, depth: np.ndarray) -> tuple[bool, int, np.ndarray | None]:
        """이 한 프레임만 보고 판단(히스테리시스 적용 전)."""
        base = self.baseline
        if base.shape != depth.shape:
            base = cv2.resize(base, (depth.shape[1], depth.shape[0]),
                              interpolation=cv2.INTER_NEAREST)

        d = depth.astype(np.int32)
        b = base.astype(np.int32)
        valid = (d > 0) & (d < self.conf.max_valid_mm) & (b > 0)
        higher = (b - d) > self.conf.intrusion_mm       # 테이블보다 이만큼 위로 솟음
        band = self._band_for(depth.shape)       # 임계값도 여기서 같이 정해진다
        hits = (higher & valid & (band > 0))
        count = int(np.count_nonzero(hits))
        return count >= self.threshold_px, count, hits

    def update(self, depth: np.ndarray | None) -> bool:
        """깊이 프레임 하나로 상태를 갱신하고 현재 침입 여부를 돌려준다.

        들어올 때보다 나갈 때를 더 까다롭게 본다(clear_frames > enter_frames).
        깜빡임 한 번에 로봇이 섰다 갔다 하면 그게 더 위험하기 때문이다.
        """
        if not self.enabled or depth is None:
            return self.intruded

        hit, count, _ = self._raw_intrusion(depth)
        self.last_pixels = count

        if hit:
            self._hot += 1
            self._cold = 0
            if self._hot >= self.conf.enter_frames:
                self.intruded = True
        else:
            self._cold += 1
            self._hot = 0
            if self._cold >= self.conf.clear_frames:
                self.intruded = False
        return self.intruded

    def reset(self) -> None:
        self.intruded = False
        self._hot = self._cold = 0

    # ── 시각화 ────────────────────────────────────────────────────────────
    def draw(self, bgr: np.ndarray, depth: np.ndarray | None = None) -> np.ndarray:
        """경계 띠와 침입 상태를 그려 넣는다."""
        out = bgr
        if self.zone is None:
            return out
        poly = self.zone.to_pixels(out.shape)
        color = (0, 0, 255) if self.intruded else (0, 200, 0)
        cv2.polylines(out, [poly], True, color, 2)

        if self.enabled and depth is not None and self.intruded:
            _hit, _count, hits = self._raw_intrusion(depth)
            if hits is not None:
                overlay = cv2.resize(hits.astype(np.uint8) * 255,
                                     (out.shape[1], out.shape[0]),
                                     interpolation=cv2.INTER_NEAREST)
                out[overlay > 0] = (0, 0, 255)

        if not self.enabled:
            text = "SAFETY OFF — setup_zone 을 먼저 실행하세요"
        elif self.intruded:
            text = f"!! 사람 감지 ({self.last_pixels}px) — 정지"
        else:
            text = f"안전 ({self.last_pixels}px)"
        for c, tk in (((0, 0, 0), 3), (color if self.enabled else (0, 200, 255), 1)):
            cv2.putText(out, text, (8, out.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, tk, cv2.LINE_AA)
        return out


def _monitor() -> None:
    """로봇 없이 감지만 확인한다. 손을 넣었다 뺐다 하며 반응을 본다."""
    from hp60c_camera import CameraReader

    monitor = SafetyMonitor.load()
    if not monitor.enabled:
        print("경계선/기준선이 없습니다 — 먼저: python -m sorting.setup_zone")
    print("q 로 종료. 손을 작업면 바깥 띠에 넣어 보세요.")

    with CameraReader() as cam:
        last = 0
        while True:
            rgb, depth, last = cam.read_blocking(last)
            if rgb is None:
                continue
            monitor.update(depth)
            cv2.imshow("safety (q=quit)", monitor.draw(rgb.copy(), depth))
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    _monitor()
