# -*- coding: utf-8 -*-
"""main_window.py — 창 하나에 카메라·3D 트윈·제어를 모아 배선한다.

여기서 하는 일은 배선뿐이다. 검출도, 로봇 제어도, 안전 판단도 전부 워커
스레드가 하고, 이 파일은 그 결과를 받아 위젯에 꽂아준다. 메인 스레드가
붙잡히면 E-STOP 버튼조차 안 눌리기 때문에 그 경계를 엄격히 지킨다.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QMainWindow, QMessageBox,
                               QSplitter, QVBoxLayout, QWidget)

from ..mapping import Mapper
from ..safety import SafetyMonitor
from ..stats import SessionStats
from .panel import ControlPanel
from .views import CameraView, TwinView
from .workers import CameraWorker, PipelineWorker

_EVENT_TEXT = {
    "sorted": lambda d: f"✓ {d['color']} 공 분류 ({d['seconds']:.1f}s)",
    "skip": lambda d: f"건너뜀: {d['reason']}",
    "grasp_failed": lambda d: f"파지 실패 — 재시도 {d['attempt'] + 1}",
    "target": lambda d: f"→ {d['color']} 공 ({d['robot'][0]:+.3f}, {d['robot'][1]:+.3f})",
    "error": lambda d: f"오류: {d['message']}",
    "complete": lambda d: f"■ 분류 완료 — 총 {d['count']}개",
}


class MainWindow(QMainWindow):
    def __init__(self, real: bool = False, slow: bool = False,
                 camera_source=None):
        super().__init__()
        self.setWindowTitle(
            f"SO-ARM101 색 분류 스테이션 — {'실물' if real else '시뮬'}")
        self.resize(1280, 720)

        self.stats = SessionStats()
        self.safety = SafetyMonitor.load()
        self.mapper = self._load_mapper()

        self._build_ui()
        self._start_workers(real, slow, camera_source)

    # ── 화면 ──────────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        central = QWidget()
        root = QHBoxLayout(central)

        views = QSplitter(Qt.Horizontal)
        self.camera_view = CameraView()
        self.camera_view.safety = self.safety
        self.camera_view.mapper = self.mapper
        self.twin_view = TwinView()

        for widget, title in ((self.camera_view, "카메라 — 검출 결과"),
                              (self.twin_view, "3D 디지털 트윈")):
            wrap = QWidget()
            v = QVBoxLayout(wrap)
            v.setContentsMargins(0, 0, 0, 0)
            label = QLabel(title)
            label.setStyleSheet("color:#8b949e; padding:2px;")
            v.addWidget(label)
            v.addWidget(widget)
            views.addWidget(wrap)
        views.setSizes([760, 460])

        self.panel = ControlPanel()
        self.panel.startClicked.connect(self._on_start)
        self.panel.pauseClicked.connect(self._on_pause)
        self.panel.estopClicked.connect(self._on_estop)
        self.panel.resetClicked.connect(self._on_reset)
        self.panel.homeClicked.connect(lambda: self.pipeline_worker.request_home())
        self.panel.jogRequested.connect(
            lambda j, d: self.pipeline_worker.request_jog(j, d))

        root.addWidget(views, stretch=1)
        root.addWidget(self.panel)
        self.setCentralWidget(central)

        self.statusBar().showMessage("준비 중…")

        if self.mapper is None:
            self.panel.log("⚠ H.npy 없음 — 로봇 이동 불가 (calibrate 먼저)")

    def _load_mapper(self) -> Mapper | None:
        try:
            return Mapper.load()
        except SystemExit:
            return None            # 캘리브레이션 전에도 화면은 뜨게 한다

    # ── 스레드 ────────────────────────────────────────────────────────────
    def _start_workers(self, real: bool, slow: bool, camera_source) -> None:
        self.camera_worker = CameraWorker(source=camera_source, safety=self.safety)
        self.camera_worker.sceneReady.connect(self._on_scene)
        self.camera_worker.failed.connect(self._on_camera_failed)
        self.camera_worker.start()

        self.pipeline_worker = PipelineWorker(
            real=real, slow=slow,
            obs_source=self.camera_worker.latest,
            mapper=self.mapper, stats=self.stats)
        self.pipeline_worker.stateChanged.connect(self.panel.set_state)
        self.pipeline_worker.jointsChanged.connect(self.twin_view.update_joints)
        self.pipeline_worker.event.connect(self._on_event)
        self.pipeline_worker.ready.connect(
            lambda: self.statusBar().showMessage("준비 완료 — 분류 시작을 누르세요"))
        self.pipeline_worker.failed.connect(self._on_robot_failed)
        self.pipeline_worker.start()

        self._stats_timer = QTimer(self)
        self._stats_timer.timeout.connect(self._refresh_stats)
        self._stats_timer.start(500)

    # ── 신호 처리 ─────────────────────────────────────────────────────────
    def _on_scene(self, payload) -> None:
        rgb, scene, _balls, intruded, depth = payload
        self.camera_view.update_scene(
            rgb, scene, intruded, depth,
            fps=self.camera_worker.fps,
            latency_ms=self.camera_worker.latency_ms)
        self.panel.set_safety(self.safety.enabled, intruded)

    def _on_event(self, name: str, data: dict) -> None:
        formatter = _EVENT_TEXT.get(name)
        if formatter:
            self.panel.log(formatter(data))

    def _refresh_stats(self) -> None:
        self.panel.set_stats(self.stats.summary_lines())
        robot = getattr(self.pipeline_worker, "robot", None)
        if robot is not None:
            self.panel.set_estopped(robot.estopped)

    def _on_start(self) -> None:
        if self.mapper is None:
            QMessageBox.warning(
                self, "캘리브레이션 필요",
                "data/H.npy 가 없습니다.\n\n먼저 실행하세요:\n"
                "    python -m sorting.calibrate")
            return
        if not self.safety.enabled:
            answer = QMessageBox.question(
                self, "안전감시가 꺼져 있습니다",
                "안전 경계선이 설정되지 않아 사람 감지가 동작하지 않습니다.\n"
                "그래도 시작할까요?\n\n(권장: 먼저 python -m sorting.setup_zone)",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        self.pipeline_worker.request_start()

    def _on_pause(self) -> None:
        if self.panel.pause_btn.isChecked():
            self.pipeline_worker.request_pause()
        else:
            self.pipeline_worker.request_resume()

    def _on_estop(self) -> None:
        self.pipeline_worker.request_estop()
        self.panel.pause_btn.setChecked(True)
        self.panel.set_state("paused", "비상정지 — 해제 후 재시작")
        self.statusBar().showMessage("비상정지됨")

    def _on_reset(self) -> None:
        self.pipeline_worker.request_release_estop()
        self.pipeline_worker.request_resume()
        self.panel.pause_btn.setChecked(False)
        self.statusBar().showMessage("정지 해제")

    def _on_camera_failed(self, message: str) -> None:
        self.statusBar().showMessage(f"카메라 오류: {message}")
        QMessageBox.critical(
            self, "카메라를 열 수 없습니다",
            f"{message}\n\n브리지가 실행 중인지 확인하세요:\n"
            "    ./hp60c-camera/scripts/start_bridge.sh")

    def _on_robot_failed(self, message: str) -> None:
        self.statusBar().showMessage(f"로봇 오류: {message}")
        self.panel.log(f"로봇 오류: {message}")

    # ── 종료 ──────────────────────────────────────────────────────────────
    def closeEvent(self, event):        # noqa: N802  (Qt 규약)
        """워커를 세우고 통계를 남긴 뒤 닫는다."""
        self._stats_timer.stop()
        self.pipeline_worker.stop()
        self.camera_worker.stop()
        self.twin_view.close_renderer()

        if self.stats.total:
            try:
                json_path, _csv = self.stats.save()
                print("통계 저장:", json_path)
            except Exception as exc:
                print("통계 저장 실패:", exc)
        super().closeEvent(event)
