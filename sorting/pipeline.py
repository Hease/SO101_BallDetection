# -*- coding: utf-8 -*-
"""pipeline.py — 분류 상태머신. 이 시스템이 실제로 '무엇을 하는지'가 여기 있다.

    SCAN → 공급구역에서 공 하나 고르기
    PICK → 옆으로 접근해 집기
    VERIFY → 진짜 잡혔나 확인 (아니면 재시도)
    PLACE → **그 순간 다시 검출한** bin 좌표로 옮겨 놓기
    → 공이 없어질 때까지 반복 → CEREMONY

PLACE 에서 bin 좌표를 매번 새로 읽는 게 이 과제의 핵심이다. 어디에도 bin 위치를
저장해두지 않으므로, 사이클 도중 사람이 bin 을 옮겨도 다음 공은 새 자리로 간다.

Qt 에 의존하지 않는다 — 콜백(on_event)으로만 바깥과 이야기한다. 덕분에 GUI 없이
테스트할 수 있고, GUI 는 이 콜백을 시그널로 옮기기만 하면 된다.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable

from . import config as cfg
from .mapping import Mapper
from .robot import OutOfReach, RobotController
from .vision import Ball, Scene


class State(str, Enum):
    IDLE = "idle"
    SCAN = "scan"
    PICK = "pick"
    VERIFY = "verify"
    PLACE = "place"
    PAUSED = "paused"        # 사람 손 감지로 멈춘 상태
    CEREMONY = "ceremony"
    DONE = "done"


@dataclass
class Observation:
    """카메라 스레드가 파이프라인에 넘겨주는 최신 관측."""
    scene: Scene
    balls: list[Ball] = field(default_factory=list)   # 트래커가 확정한 공만
    intruded: bool = False


ObsSource = Callable[[], Observation]
EventSink = Callable[[str, dict], None]


class SortingPipeline:
    """공이 없어질 때까지 집어서 색깔 맞는 bin 에 넣는 루프.

    run() 은 블로킹이다 — 워커 스레드에서 돌린다.
    """

    def __init__(self, robot: RobotController, mapper: Mapper,
                 obs_source: ObsSource, sound=None, stats=None,
                 on_event: EventSink | None = None,
                 conf: cfg.RobotConfig | None = None):
        self.robot = robot
        self.mapper = mapper
        self.obs_source = obs_source
        self.sound = sound
        self.stats = stats
        self.on_event = on_event or (lambda name, data: None)
        self.conf = conf or cfg.ROBOT

        self.state = State.IDLE
        self._stop = False
        self._pause_requested = False
        # 끝내 못 집은 공의 로봇좌표. 다시 고르지 않으려고 기억해둔다 —
        # 없으면 집을 수 없는 공 하나에 걸려 루프가 영원히 끝나지 않는다.
        self._skipped: list[tuple[str, float, float]] = []

        # 이동 중에도 침입을 알아채도록 로봇에 훅을 건다
        self.robot.should_abort = self._should_abort

    # ── 바깥에서 거는 제어 ────────────────────────────────────────────────
    def request_stop(self) -> None:
        self._stop = True

    def request_pause(self) -> None:
        self._pause_requested = True

    def request_resume(self) -> None:
        self._pause_requested = False

    # ── 내부 ──────────────────────────────────────────────────────────────
    def _should_abort(self) -> bool:
        """이동 한복판에서 즉시 멈춰야 하는가."""
        if self._stop or self._pause_requested:
            return True
        return self.obs_source().intruded

    def _emit(self, name: str, **data) -> None:
        self.on_event(name, data)

    def _set_state(self, state: State, note: str = "") -> None:
        self.state = state
        self._emit("state", state=state.value, note=note)

    def _play(self, tune: str) -> None:
        if self.sound is not None:
            self.sound.play(tune)

    def _bin_xy(self, color: str, obs: Observation) -> tuple[float, float] | None:
        """그 색 bin 의 **현재** 로봇 좌표. 없으면 None.

        저장된 값을 쓰지 않고 방금 들어온 관측에서 뽑는다 — bin 을 옮겨도
        따라가는 이유가 정확히 이 한 줄이다.
        """
        region = obs.scene.regions.get(color)
        if region is None:
            return None
        return self.mapper.to_robot(region.cx, region.cy)

    # ── 메인 루프 ─────────────────────────────────────────────────────────
    def run(self) -> None:
        """공이 없어질 때까지 분류. 워커 스레드에서 호출한다."""
        self._stop = False
        self._set_state(State.SCAN)
        sorted_count = 0

        try:
            self.robot.home()
            while not self._stop:
                if self._handle_interrupt():
                    continue                      # 재개하면 SCAN 부터 다시

                obs = self.obs_source()
                target = self._choose_ball(obs)
                if target is None:
                    self._set_state(State.CEREMONY, f"{sorted_count}개 분류 완료")
                    self._ceremony(sorted_count)
                    self._set_state(State.DONE)
                    return

                if self._run_cycle(target, obs):
                    sorted_count += 1
        except Exception as exc:                  # 어떤 이유로든 팔은 세우고 끝낸다
            self.robot.estop()
            self._emit("error", message=str(exc))
            raise
        finally:
            self.robot.should_abort = None

    def _handle_interrupt(self) -> bool:
        """침입·일시정지 처리. 멈췄다 재개했으면 True(=다시 스캔)."""
        obs = self.obs_source()
        if not (obs.intruded or self._pause_requested):
            return False

        reason = "사람 감지" if obs.intruded else "일시정지"
        self.robot.estop()
        self._set_state(State.PAUSED, reason)
        if obs.intruded:
            self._play("warn")

        while not self._stop:
            obs = self.obs_source()
            if not obs.intruded and not self._pause_requested:
                break
            time.sleep(0.05)

        if self._stop:
            return True

        self._play("resume")
        self.robot.release_estop()
        # 재개는 SCAN 부터 — 사람이 손을 넣었다면 공이나 bin 이 움직였을 수 있다.
        # 못 집던 공을 사람이 바로 놓아줬을 수도 있으니 포기 목록도 비운다.
        self._skipped.clear()
        self._set_state(State.SCAN, "재개")
        if self.stats is not None:
            self.stats.record_pause()
        return True

    def _is_skipped(self, x: float, y: float, color: str) -> bool:
        r = self.conf.skip_radius_m
        return any(c == color and (x - sx) ** 2 + (y - sy) ** 2 <= r * r
                   for c, sx, sy in self._skipped)

    def _choose_ball(self, obs: Observation) -> Ball | None:
        """다음에 집을 공. 로봇 베이스에 가까운 것부터 — 팔이 덜 움직인다.

        끝내 못 집었던 자리의 공은 후보에서 뺀다. 안 그러면 집을 수 없는 공
        하나를 무한히 다시 고르느라 나머지 공도 영영 처리하지 못한다.
        """
        candidates = []
        for b in obs.balls:
            x, y = self.mapper.to_robot(b.u, b.v)
            if self._is_skipped(x, y, b.color):
                continue
            candidates.append((x * x + y * y, b))
        if not candidates:
            return None
        return min(candidates, key=lambda t: t[0])[1]

    def _run_cycle(self, ball: Ball, obs: Observation) -> bool:
        """공 하나를 집어서 놓기까지. 성공하면 True."""
        started = time.monotonic()
        bx, by = self.mapper.to_robot(ball.u, ball.v)
        self._emit("target", color=ball.color, pixel=ball.uv, robot=(bx, by))

        bin_color = cfg.PLACE_MAP.get(ball.color, ball.color)
        if self._bin_xy(bin_color, obs) is None:
            self._set_state(State.SCAN, f"{bin_color} bin 이 안 보임 — 대기")
            time.sleep(0.5)
            return False

        # ── 집기 (실패하면 재시도) ──
        for attempt in range(self.conf.max_retries + 1):
            self._set_state(State.PICK,
                            f"{ball.color} 공" + (f" (재시도 {attempt})" if attempt else ""))
            try:
                self.robot.pick((bx, by), attempt=attempt)
            except OutOfReach as exc:
                self._emit("skip", reason=f"팔 범위 밖: {exc}")
                self._skipped.append((ball.color, bx, by))
                if self.stats is not None:
                    self.stats.record_unreachable()
                return False

            if self.robot.estopped:
                return False

            self._set_state(State.VERIFY)
            if self.robot.holding_object():
                break
            self._emit("grasp_failed", attempt=attempt)
            if self.stats is not None:
                self.stats.record_retry()
        else:
            self._emit("skip", reason=f"{self.conf.max_retries}회 시도 후 파지 실패")
            self.robot.set_grip(self.conf.grip_open)
            self._skipped.append((ball.color, bx, by))
            if self.stats is not None:
                self.stats.record_failure(ball.color)
            return False

        # ── 놓기: bin 좌표를 지금 다시 읽는다 (여기가 요구사항의 핵심) ──
        fresh = self.obs_source()
        bin_xy = self._bin_xy(bin_color, fresh) or self._bin_xy(bin_color, obs)
        if bin_xy is None:
            self._emit("skip", reason=f"{bin_color} bin 을 놓칠 때 잃어버림")
            return False

        self._set_state(State.PLACE, f"{bin_color} bin ({bin_xy[0]:+.3f}, {bin_xy[1]:+.3f})")
        try:
            self.robot.place(bin_xy)
        except OutOfReach as exc:
            self._emit("skip", reason=f"bin 이 팔 범위 밖: {exc}")
            return False

        if self.robot.estopped:
            return False

        elapsed = time.monotonic() - started
        self._emit("sorted", color=ball.color, seconds=elapsed)
        self._play("pick_ok")
        if self.stats is not None:
            self.stats.record_success(ball.color, elapsed)
        return True

    def _ceremony(self, count: int) -> None:
        """분류 완료 — 소리·동작·화면이 함께 터진다."""
        self._emit("ceremony", count=count)
        self._play("victory")
        try:
            self.robot.celebrate()
            self.robot.home()
        except OutOfReach:
            pass
