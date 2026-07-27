# -*- coding: utf-8 -*-
"""ceremony.py — 분류를 다 끝냈을 때 화면에 터지는 축하 효과.

부저 팡파레·로봇 승리 동작과 같은 순간에 실행돼 셋이 함께 터진다.
창 위에 얹히는 투명 오버레이라 아래 영상은 계속 보인다.
"""
from __future__ import annotations

import random

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter
from PySide6.QtWidgets import QWidget

DURATION_MS = 4000
TICK_MS = 16                  # 약 60fps
CONFETTI_COUNT = 90
GRAVITY = 0.28
COLORS = ["#f85149", "#58a6ff", "#3fb950", "#d29922", "#bc8cff", "#ff7b72"]


class _Confetto:
    """조각 하나. 던져 올렸다가 중력으로 떨어지며 회전한다."""

    __slots__ = ("x", "y", "vx", "vy", "size", "color", "spin", "angle")

    def __init__(self, width: int, height: int):
        self.x = random.uniform(0, width)
        self.y = random.uniform(-height * 0.3, 0)
        self.vx = random.uniform(-2.2, 2.2)
        self.vy = random.uniform(1.0, 4.5)
        self.size = random.uniform(6, 13)
        self.color = QColor(random.choice(COLORS))
        self.spin = random.uniform(-9, 9)
        self.angle = random.uniform(0, 360)

    def step(self) -> None:
        self.vy += GRAVITY
        self.x += self.vx
        self.y += self.vy
        self.angle += self.spin


class CeremonyOverlay(QWidget):
    """"분류 완료" 배너 + 컨페티. show_ceremony() 로 한 번 재생된다."""

    finished = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WA_NoSystemBackground)
        self.hide()

        self._pieces: list[_Confetto] = []
        self._elapsed = 0
        self._title = ""
        self._subtitle = ""

        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._tick)

    def show_ceremony(self, count: int, subtitle: str = "") -> None:
        if self.parent() is not None:
            self.setGeometry(self.parent().rect())
        self._title = f"분류 완료!  {count}개"
        self._subtitle = subtitle
        self._pieces = [_Confetto(max(self.width(), 1), max(self.height(), 1))
                        for _ in range(CONFETTI_COUNT)]
        self._elapsed = 0
        self.show()
        self.raise_()
        self._timer.start()

    def _tick(self) -> None:
        self._elapsed += TICK_MS
        for piece in self._pieces:
            piece.step()
        # 아래로 다 떨어진 조각은 위에서 다시 뿌린다 — 배너가 떠 있는 동안 계속 내린다
        if self._elapsed < DURATION_MS * 0.6:
            for piece in self._pieces:
                if piece.y > self.height() + 20:
                    piece.__init__(max(self.width(), 1), max(self.height(), 1))
        if self._elapsed >= DURATION_MS:
            self._timer.stop()
            self.hide()
            self.finished.emit()
        self.update()

    def paintEvent(self, event):        # noqa: N802  (Qt 규약)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 끝날 무렵 서서히 사라지게
        fade = 1.0
        tail = DURATION_MS * 0.25
        if self._elapsed > DURATION_MS - tail:
            fade = max(0.0, (DURATION_MS - self._elapsed) / tail)
        painter.setOpacity(fade)

        for piece in self._pieces:
            painter.save()
            painter.translate(QPointF(piece.x, piece.y))
            painter.rotate(piece.angle)
            painter.fillRect(
                QRectF(-piece.size / 2, -piece.size / 4, piece.size, piece.size / 2),
                piece.color)
            painter.restore()

        band_h = 120
        band = QRectF(0, self.height() / 2 - band_h / 2, self.width(), band_h)
        painter.fillRect(band, QColor(0, 0, 0, 165))

        painter.setPen(QColor("#f0f6fc"))
        painter.setFont(QFont("", 30, QFont.Bold))
        painter.drawText(band.adjusted(0, 8, 0, -40), Qt.AlignCenter, self._title)

        if self._subtitle:
            painter.setPen(QColor("#8b949e"))
            painter.setFont(QFont("", 12))
            painter.drawText(band.adjusted(0, 70, 0, 0), Qt.AlignHCenter | Qt.AlignTop,
                             self._subtitle)
