# -*- coding: utf-8 -*-
"""tracking.py — 프레임을 가로질러 공을 같은 공으로 알아본다.

한 프레임의 검출만 믿으면 그늘 하나에도 공이 사라졌다 나타난다. 그 상태로
로봇을 움직이면 팔이 덜덜 떨린다. 그래서 두 가지를 한다:

  · 연속 confirm_frames 번 보여야 '확정' — 한 프레임짜리 노이즈는 무시된다.
  · 위치는 EMA 로 평활 — 검출이 1~2px 씩 흔들려도 목표 좌표는 잔잔하다.

05_track.py 의 데드밴드(MARGIN)와 같은 문제의식을, 공이 여러 개인 상황으로
확장한 것이다.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from . import config as cfg
from .vision import Ball, Scene


@dataclass
class Track:
    """추적 중인 공 하나."""
    tid: int
    color: str
    u: float
    v: float
    r: float
    area: float
    seen: int = 1          # 연속 검출 횟수
    missed: int = 0        # 연속 미검출 횟수
    confirmed: bool = False

    def as_ball(self) -> Ball:
        return Ball(self.color, int(round(self.u)), int(round(self.v)),
                    int(round(self.r)), self.area)


class BallTracker:
    """최근접 매칭으로 공에 ID를 붙이고, 안정된 것만 내보낸다.

    공은 사람이 옮기지 않는 한 제자리에 있으므로 매칭은 '가장 가까운 것'
    하나로 충분하다. 색이 다르면 절대 같은 공으로 묶지 않는다.
    """

    def __init__(self, conf: cfg.TrackingConfig | None = None):
        self.conf = conf or cfg.TRACKING
        self.tracks: list[Track] = []
        self._ids = itertools.count(1)

    def reset(self) -> None:
        self.tracks.clear()

    def update(self, scene: Scene) -> list[Ball]:
        """Scene 의 검출로 트랙을 갱신하고 확정된 공 목록을 돌려준다."""
        unmatched = list(scene.balls)

        for tr in self.tracks:
            best, best_d = None, self.conf.match_radius_px
            for b in unmatched:
                if b.color != tr.color:
                    continue
                d = math.hypot(b.u - tr.u, b.v - tr.v)
                if d < best_d:
                    best, best_d = b, d

            if best is None:
                tr.missed += 1
                tr.seen = 0
                continue

            unmatched.remove(best)
            a = self.conf.ema_alpha
            tr.u += a * (best.u - tr.u)        # EMA 평활 — 목표가 잔잔해진다
            tr.v += a * (best.v - tr.v)
            tr.r += a * (best.r - tr.r)
            tr.area = best.area
            tr.missed = 0
            tr.seen += 1
            if tr.seen >= self.conf.confirm_frames:
                tr.confirmed = True

        for b in unmatched:                    # 처음 보는 공
            self.tracks.append(Track(next(self._ids), b.color,
                                     float(b.u), float(b.v), float(b.r), b.area))

        self.tracks = [t for t in self.tracks
                       if t.missed < self.conf.forget_frames]

        return [t.as_ball() for t in self.tracks if t.confirmed]

    def confirmed_tracks(self) -> list[Track]:
        return [t for t in self.tracks if t.confirmed]
