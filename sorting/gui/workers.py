# -*- coding: utf-8 -*-
"""workers.py — 백그라운드 스레드 둘. GUI 를 얼리지 않기 위한 전부가 여기 있다.

CameraWorker   : 카메라를 읽고 → 검출 → 트래킹 → 안전감시. 최신 관측을 들고 있다.
PipelineWorker : 상태머신을 돌린다. **로봇 시리얼을 만지는 유일한 스레드.**

시리얼 규칙이 중요하다. driver_sdk 는 sync write 없이 관절마다 개별 write 를 하므로
두 스레드가 동시에 쓰면 패킷이 섞인다. 그래서 RobotController 는 PipelineWorker
안에서만 만들고 만진다.
"""
from __future__ import annotations

import time

from PySide6.QtCore import QMutex, QMutexLocker, QThread, Signal

from ..pipeline import Observation, SortingPipeline
from ..safety import SafetyMonitor
from ..tracking import BallTracker
from ..vision import Scene, observe


class CameraWorker(QThread):
    """카메라 프레임을 읽어 Scene 으로 바꿔 내보낸다.

    파이프라인은 이 스레드를 기다리지 않는다 — latest() 로 '지금 최신'만 집어간다.
    로봇이 한 동작을 하는 동안 카메라는 계속 돌아야 침입을 제때 잡을 수 있다.
    """

    sceneReady = Signal(object)      # (rgb, Scene, balls, intruded, fps)
    failed = Signal(str)

    def __init__(self, source=None, safety: SafetyMonitor | None = None, parent=None):
        super().__init__(parent)
        self.source = source                  # None 이면 실제 카메라
        self.safety = safety or SafetyMonitor.load()
        self.tracker = BallTracker()
        self._running = True
        self._mutex = QMutex()
        self._latest = Observation(scene=Scene())
        self._fps = 0.0
        self._latency_ms = 0.0

    # ── 파이프라인이 부르는 쪽 ────────────────────────────────────────────
    def latest(self) -> Observation:
        with QMutexLocker(self._mutex):
            return self._latest

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def latency_ms(self) -> float:
        return self._latency_ms

    # ── 스레드 본체 ───────────────────────────────────────────────────────
    def run(self) -> None:
        try:
            if self.source is not None:
                self._loop(self.source)
            else:
                from hp60c_camera import CameraReader
                with CameraReader() as cam:
                    self._loop(cam)
        except Exception as exc:
            self.failed.emit(str(exc))

    def _loop(self, cam) -> None:
        last_id = 0
        frame_times: list[float] = []

        while self._running:
            t0 = time.monotonic()
            rgb, depth, last_id = cam.read_blocking(last_id)
            if rgb is None:
                continue

            scene = observe(rgb, frame_id=last_id)
            balls = self.tracker.update(scene)
            intruded = self.safety.update(depth)

            self._latency_ms = (time.monotonic() - t0) * 1000.0
            frame_times.append(t0)
            if len(frame_times) > 15:
                frame_times.pop(0)
            if len(frame_times) > 1:
                span = frame_times[-1] - frame_times[0]
                self._fps = (len(frame_times) - 1) / span if span > 0 else 0.0

            with QMutexLocker(self._mutex):
                self._latest = Observation(scene=scene, balls=balls, intruded=intruded)

            self.sceneReady.emit((rgb, scene, balls, intruded, depth))

    def stop(self) -> None:
        self._running = False
        self.wait(2000)


class PipelineWorker(QThread):
    """분류 상태머신을 돌리는 스레드. 로봇을 여기서 만들고 여기서만 만진다."""

    stateChanged = Signal(str, str)          # (상태, 설명)
    jointsChanged = Signal(object)           # 관절각 6개 → 3D 트윈
    event = Signal(str, object)              # 그 밖의 사건 (sorted/skip/complete/...)
    ready = Signal()
    failed = Signal(str)

    def __init__(self, real: bool, slow: bool, obs_source, mapper,
                 stats=None, parent=None):
        super().__init__(parent)
        self.real = real
        self.slow = slow
        self.obs_source = obs_source
        self.mapper = mapper
        self.stats = stats

        self.robot = None
        self.pipeline: SortingPipeline | None = None
        self._start_requested = False
        self._running = True
        self._jog_queue: list[tuple[int, float]] = []
        self._mutex = QMutex()

    # ── 메인 스레드에서 거는 요청 (직접 로봇을 만지지 않는다) ─────────────
    def request_start(self) -> None:
        self._start_requested = True

    def request_pause(self) -> None:
        if self.pipeline:
            self.pipeline.request_pause()

    def request_resume(self) -> None:
        if self.pipeline:
            self.pipeline.request_resume()

    def request_estop(self) -> None:
        """E-STOP 만은 예외적으로 즉시 실행한다.

        큐에 넣고 워커가 집어가길 기다리면, 그 워커가 마침 이동 대기 중일 때
        정지가 늦는다. 안전 기능이 '차례를 기다리는' 건 말이 안 된다.
        set_position 은 개별 패킷이라 진행 중인 명령과 섞여도 손상되지 않는다.
        """
        if self.robot:
            self.robot.estop()
        if self.pipeline:
            self.pipeline.request_pause()

    def request_release_estop(self) -> None:
        if self.robot:
            self.robot.release_estop()

    def request_jog(self, joint: int, delta: float) -> None:
        with QMutexLocker(self._mutex):
            self._jog_queue.append((joint, delta))

    def request_home(self) -> None:
        with QMutexLocker(self._mutex):
            self._jog_queue.append((-1, 0.0))     # -1 = home 신호

    # ── 스레드 본체 ───────────────────────────────────────────────────────
    def run(self) -> None:
        from ..robot import RobotController

        try:
            self.robot = RobotController(real=self.real, slow=self.slow)
        except Exception as exc:
            self.failed.emit(str(exc))
            return

        self.pipeline = SortingPipeline(
            self.robot, self.mapper, self.obs_source,
            stats=self.stats, on_event=self._on_event)
        self.ready.emit()

        while self._running:
            self._drain_jogs()
            self.jointsChanged.emit(self.robot.joint_angles_deg())

            if self._start_requested:
                self._start_requested = False
                try:
                    self.pipeline.run()
                except Exception as exc:
                    self.failed.emit(str(exc))
            else:
                self.msleep(50)

    def _drain_jogs(self) -> None:
        with QMutexLocker(self._mutex):
            jobs, self._jog_queue = self._jog_queue, []
        for joint, delta in jobs:
            try:
                if joint < 0:
                    self.robot.home()
                else:
                    self.robot.jog(joint, delta)
            except Exception as exc:
                self.event.emit("error", {"message": str(exc)})

    def _on_event(self, name: str, data: dict) -> None:
        if name == "state":
            self.stateChanged.emit(data.get("state", ""), data.get("note", ""))
        else:
            self.event.emit(name, data)
        if self.robot is not None:
            self.jointsChanged.emit(self.robot.joint_angles_deg())

    def stop(self) -> None:
        self._running = False
        if self.pipeline:
            self.pipeline.request_stop()
        if self.robot:
            self.robot.close()
        self.wait(3000)
