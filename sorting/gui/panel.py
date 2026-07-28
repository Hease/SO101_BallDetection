# -*- coding: utf-8 -*-
"""panel.py — 오른쪽 제어 패널: 상태 · 통계 · 조깅.

조깅 버튼은 **수동 모드에서만** 활성화된다. 자동 분류가 도는 중에 사람이
관절을 밀면 파이프라인이 계산해둔 좌표와 실제 자세가 어긋나기 때문이다.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QButtonGroup, QFrame, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QRadioButton,
                               QSlider, QVBoxLayout, QWidget)

JOINT_NAMES = ["베이스", "어깨", "팔꿈치", "손목", "회전", "그리퍼"]
JOG_STEP_DEG = 5.0
GRIP_STEP_PCT = 10.0

_STATE_KO = {
    "idle": "대기", "scan": "탐색 중", "pick": "집는 중", "verify": "확인 중",
    "place": "놓는 중", "paused": "일시정지",
    "done": "완료",
}
_STATE_COLOR = {
    "paused": "#e5534b", "done": "#3fb950",
    "idle": "#8b949e",
}


class ControlPanel(QWidget):
    """상태 표시 + 버튼. 실제 동작은 시그널로만 알린다(로봇을 직접 안 만진다)."""

    startClicked = Signal()
    pauseClicked = Signal()
    estopClicked = Signal()
    resetClicked = Signal()
    homeClicked = Signal()
    jogRequested = Signal(int, float)
    modeChanged = Signal(bool)          # True = 자동
    injectDelayChanged = Signal(int)    # 데모용 지연 주입(ms)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(300)
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        layout.addWidget(self._build_status())
        layout.addWidget(self._build_buttons())
        layout.addWidget(self._build_mode())
        layout.addWidget(self._build_delay())
        layout.addWidget(self._build_jog())
        layout.addWidget(self._build_stats())
        layout.addStretch(1)
        self._set_manual_enabled(False)

    # ── 구성 ──────────────────────────────────────────────────────────────
    def _build_status(self) -> QWidget:
        box = QGroupBox("상태")
        v = QVBoxLayout(box)
        self.state_label = QLabel("대기")
        self.state_label.setStyleSheet("font-size:20px; font-weight:bold;")
        self.note_label = QLabel("")
        self.note_label.setWordWrap(True)
        self.note_label.setStyleSheet("color:#8b949e;")
        self.safety_label = QLabel("")
        self.safety_label.setWordWrap(True)
        self.latency_label = QLabel("지연: —")
        self.latency_label.setStyleSheet("color:#8b949e;")
        self.calib_label = QLabel("")
        self.calib_label.setWordWrap(True)
        for w in (self.state_label, self.note_label, self.safety_label,
                  self.latency_label, self.calib_label):
            v.addWidget(w)
        return box

    def _build_buttons(self) -> QWidget:
        box = QGroupBox("실행")
        v = QVBoxLayout(box)

        self.start_btn = QPushButton("▶  분류 시작")
        self.start_btn.setStyleSheet("padding:10px; font-weight:bold;")
        self.start_btn.clicked.connect(self.startClicked)

        self.pause_btn = QPushButton("⏸  일시정지")
        self.pause_btn.setCheckable(True)
        self.pause_btn.clicked.connect(self.pauseClicked)

        self.estop_btn = QPushButton("■  비상정지 (E-STOP)   [Space]")
        self.estop_btn.setStyleSheet(
            "background:#8b1a1a; color:white; padding:14px; font-size:15px;"
            "font-weight:bold; border-radius:4px;")
        self.estop_btn.clicked.connect(self.estopClicked)

        self.reset_btn = QPushButton("정지 해제")
        self.reset_btn.clicked.connect(self.resetClicked)
        self.reset_btn.setEnabled(False)

        for w in (self.start_btn, self.pause_btn, self.estop_btn, self.reset_btn):
            v.addWidget(w)
        return box

    def _build_mode(self) -> QWidget:
        box = QGroupBox("모드")
        h = QHBoxLayout(box)
        self.auto_radio = QRadioButton("자동")
        self.manual_radio = QRadioButton("수동")
        self.auto_radio.setChecked(True)
        group = QButtonGroup(self)
        group.addButton(self.auto_radio)
        group.addButton(self.manual_radio)
        self.auto_radio.toggled.connect(self._on_mode)
        h.addWidget(self.auto_radio)
        h.addWidget(self.manual_radio)
        return box

    def _build_delay(self) -> QWidget:
        """데모용 지연 주입. 실제 지연과 같은 경로를 타므로 보호장치가 진짜로 돈다."""
        box = QGroupBox("지연 주입 (데모)")
        v = QVBoxLayout(box)

        self.delay_slider = QSlider(Qt.Horizontal)
        self.delay_slider.setRange(0, 2500)
        self.delay_slider.setSingleStep(50)
        self.delay_slider.setTickInterval(500)
        self.delay_slider.setTickPosition(QSlider.TicksBelow)
        self.delay_slider.valueChanged.connect(self._on_delay)

        self.delay_label = QLabel("주입 0ms — 정상")
        self.delay_label.setStyleSheet("font-family:monospace;")
        v.addWidget(self.delay_slider)
        v.addWidget(self.delay_label)
        return box

    def _on_delay(self, ms: int) -> None:
        self.injectDelayChanged.emit(ms)

    def set_latency(self, level_label: str, text: str, color: str) -> None:
        """워치독 상태를 표시한다. 무엇이 얼마나 느린지까지 보여준다."""
        self.latency_label.setText(f"지연: {text}")
        self.latency_label.setStyleSheet(f"color:{color};")
        injected = self.delay_slider.value()
        self.delay_label.setText(
            f"주입 {injected}ms — {level_label}" if injected else f"주입 0ms — {level_label}")

    def set_calibration(self, missing: int, workspace_ok: bool) -> None:
        """아직 안 잰 값이 몇 개인지, 작업영역이 있는지 알린다."""
        if not workspace_ok:
            self.calib_label.setText("⚠ 작업영역 미측정 — teach limits 필요")
            self.calib_label.setStyleSheet("color:#e5534b;")
        elif missing:
            self.calib_label.setText(f"⚠ 미측정 값 {missing}개 (calib report)")
            self.calib_label.setStyleSheet("color:#d29922;")
        else:
            self.calib_label.setText("● 캘리브레이션 완료")
            self.calib_label.setStyleSheet("color:#3fb950;")

    def _build_jog(self) -> QWidget:
        box = QGroupBox("수동 조깅")
        grid = QGridLayout(box)
        self._jog_widgets: list[QWidget] = []

        for i, name in enumerate(JOINT_NAMES):
            step = GRIP_STEP_PCT if i == 5 else JOG_STEP_DEG
            label = QLabel(name)
            minus = QPushButton("−")
            plus = QPushButton("+")
            for btn, delta in ((minus, -step), (plus, +step)):
                btn.setFixedWidth(36)
                btn.clicked.connect(
                    lambda _checked=False, j=i, d=delta: self.jogRequested.emit(j, d))
            grid.addWidget(label, i, 0)
            grid.addWidget(minus, i, 1)
            grid.addWidget(plus, i, 2)
            self._jog_widgets += [label, minus, plus]

        home = QPushButton("홈 자세로")
        home.clicked.connect(self.homeClicked)
        grid.addWidget(home, len(JOINT_NAMES), 0, 1, 3)
        self._jog_widgets.append(home)
        return box

    def _build_stats(self) -> QWidget:
        box = QGroupBox("통계")
        v = QVBoxLayout(box)
        self.stats_labels = [QLabel("—") for _ in range(3)]
        for label in self.stats_labels:
            label.setStyleSheet("font-family:monospace;")
            v.addWidget(label)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        v.addWidget(line)

        self.log_label = QLabel("")
        self.log_label.setWordWrap(True)
        self.log_label.setAlignment(Qt.AlignTop)
        self.log_label.setMinimumHeight(70)
        self.log_label.setStyleSheet("color:#8b949e; font-size:11px;")
        v.addWidget(self.log_label)
        return box

    # ── 상태 반영 ─────────────────────────────────────────────────────────
    def _on_mode(self, auto: bool) -> None:
        self._set_manual_enabled(not auto)
        self.modeChanged.emit(auto)

    def _set_manual_enabled(self, enabled: bool) -> None:
        for w in self._jog_widgets:
            w.setEnabled(enabled)
        self.start_btn.setEnabled(not enabled)

    def set_state(self, state: str, note: str = "") -> None:
        self.state_label.setText(_STATE_KO.get(state, state))
        self.state_label.setStyleSheet(
            f"font-size:20px; font-weight:bold; color:{_STATE_COLOR.get(state, '#c9d1d9')};")
        self.note_label.setText(note)
        if state in ("done", "idle"):
            self.pause_btn.setChecked(False)

    def set_estopped(self, estopped: bool) -> None:
        self.reset_btn.setEnabled(estopped)

    def set_safety(self, enabled: bool, intruded: bool) -> None:
        if not enabled:
            self.safety_label.setText("⚠ 안전감시 꺼짐 — setup_zone 을 먼저 실행하세요")
            self.safety_label.setStyleSheet("color:#d29922;")
        elif intruded:
            self.safety_label.setText("■ 사람 감지 — 정지")
            self.safety_label.setStyleSheet("color:#e5534b; font-weight:bold;")
        else:
            self.safety_label.setText("● 안전")
            self.safety_label.setStyleSheet("color:#3fb950;")

    def set_stats(self, lines: list[str]) -> None:
        for label, text in zip(self.stats_labels, lines):
            label.setText(text)

    def log(self, text: str) -> None:
        old = self.log_label.text().splitlines()
        self.log_label.setText("\n".join((old + [text])[-4:]))
