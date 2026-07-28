# -*- coding: utf-8 -*-
"""solder.py — 납땜 작업 상태머신.

    SCAN     보드 위에서 아직 안 한 납땜점을 찾는다
    APPROACH 그 점 **위로** 이동 (옆으로 움직일 땐 항상 안전 높이)
    DESCEND  수직으로 내려 인두를 댄다
    DWELL    체류 — 납이 녹는 시간만큼 머문다
    RETRACT  수직으로 들어올린다
    → 남은 점이 없을 때까지 반복

**더미(비가열) 인두로 동작만 재현한다.** 실제 가열된 인두를 이 팔에 물리면
반복정밀도(서보 기반)가 납땜에 못 미치는 것과 별개로 화상·화재 위험이 있다.
동작·정밀도·안전영역은 그대로 보여주면서 그 위험만 뺀 구성이다.

이 작업이 공 분류보다 안전장치를 잘 드러내는 이유가 있다. 인두 끝은 **옆으로
끌면 안 된다** — 보드를 긁고 부품을 밀어낸다. 그래서 "옆으로 갈 때는 반드시
안전 높이" 라는 규칙이 생기고, 그건 말이 아니라 코드로 강제해야 한다.
그리고 체류(DWELL) 중에 사람 손이 들어오면 **그 2초를 다 기다리면 안 된다.**
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable

from . import config as cfg
from .mapping import Mapper
from .robot import OutOfReach
from .vision import Ball, Scene


class State(str, Enum):
    IDLE = "idle"
    SCAN = "scan"
    APPROACH = "approach"
    DESCEND = "descend"
    DWELL = "dwell"          # 인두를 대고 있는 중
    RETRACT = "retract"
    PAUSED = "paused"
    DONE = "done"


@dataclass
class Observation:
    """카메라 스레드가 넘겨주는 최신 관측 (분류 작업과 같은 모양)."""
    scene: Scene
    balls: list[Ball] = field(default_factory=list)   # = 확정된 납땜점 마커
    intruded: bool = False


ObsSource = Callable[[], Observation]
EventSink = Callable[[str, dict], None]


class SolderPipeline:
    """마커로 표시된 납땜점을 하나씩 찾아가 인두를 대는 루프.

    안전 배선은 분류 작업과 **완전히 같다** — 침입 감지·딜레이 워치독·작업영역·
    E-STOP 이 전부 그대로 붙는다. 바뀐 것은 '무엇을 하는가'뿐이다.
    """

    def __init__(self, robot, mapper: Mapper, obs_source: ObsSource,
                 stats=None, on_event: EventSink | None = None,
                 conf: cfg.SolderConfig | None = None, watchdog=None):
        self.robot = robot
        self.mapper = mapper
        self.obs_source = obs_source
        self.stats = stats
        self.on_event = on_event or (lambda name, data: None)
        self.conf = conf or cfg.SOLDER
        self.watchdog = watchdog

        self.state = State.IDLE
        self._stop = False
        self._pause_requested = False
        # 이미 납땜한 점 (색, x, y). 마커는 그대로 남아 있으므로 우리가 기억해야
        # 한다 — 안 그러면 같은 점을 영원히 다시 납땜한다.
        self._done: list[tuple[str, float, float]] = []

        self.robot.should_abort = self._should_abort

    # ── 바깥 제어 ─────────────────────────────────────────────────────────
    def request_stop(self) -> None:
        self._stop = True

    def request_pause(self) -> None:
        self._pause_requested = True

    def request_resume(self) -> None:
        self._pause_requested = False

    # ── 내부 ──────────────────────────────────────────────────────────────
    def _should_abort(self) -> bool:
        """이동·체류 한복판에서 즉시 멈춰야 하는가."""
        if self._stop or self._pause_requested:
            return True
        if self.watchdog is not None and self.watchdog.should_abort():
            return True
        return self.obs_source().intruded

    def _emit(self, name: str, **data) -> None:
        self.on_event(name, data)

    def _set_state(self, state: State, note: str = "") -> None:
        self.state = state
        self._emit("state", state=state.value, note=note)

    def _is_done(self, x: float, y: float, color: str) -> bool:
        r = self.conf.point_radius_m
        return any(c == color and (x - dx) ** 2 + (y - dy) ** 2 <= r * r
                   for c, dx, dy in self._done)

    def _next_point(self, obs: Observation) -> Ball | None:
        """다음에 납땜할 점. 아직 안 한 것 중 베이스에 가까운 것부터.

        보드 구역이 검출되면 그 안의 점만 대상으로 한다 — 책상에 굴러다니는
        같은 색 물건을 납땜하러 가지 않게.
        """
        board = obs.scene.regions.get(self.conf.board_color)
        candidates = []
        for point in obs.balls:
            if board is not None and not board.contains(point.u, point.v):
                continue
            x, y = self.mapper.to_robot(point.u, point.v)
            if self._is_done(x, y, point.color):
                continue
            candidates.append((x * x + y * y, point))
        if not candidates:
            return None
        return min(candidates, key=lambda t: t[0])[1]

    def _dwell(self, seconds: float) -> bool:
        """인두를 댄 채 머문다. **중간에 끊길 수 있어야 한다.**

        `time.sleep(2.0)` 으로 두면 그 2초 동안 사람이 손을 넣어도 팔이
        움직이지 않는다 — 멈춘 것처럼 보이지만 실제로는 아무것도 감시하지
        않는 구간이 생긴다. 끝까지 머물렀으면 True, 중간에 끊겼으면 False.
        """
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._should_abort() or self.robot.estopped:
                return False
            time.sleep(0.02)
        return True

    # ── 메인 루프 ─────────────────────────────────────────────────────────
    def run(self) -> None:
        """남은 납땜점이 없을 때까지. 워커 스레드에서 호출한다."""
        self._stop = False
        self._set_state(State.SCAN)
        done_count = 0

        try:
            self.robot.home()
            while not self._stop:
                if self._handle_interrupt():
                    continue
                if self._wait_for_latency():
                    self._set_state(State.SCAN, "지연 회복 — 보드 재확인")
                    continue

                obs = self.obs_source()
                point = self._next_point(obs)
                if point is None:
                    self._finish(done_count)
                    return

                if self._solder_one(point):
                    done_count += 1
        except Exception as exc:
            self.robot.estop()
            self._emit("error", message=str(exc))
            raise
        finally:
            self.robot.should_abort = None

    def _handle_interrupt(self) -> bool:
        """침입·일시정지. 멈췄다 재개했으면 True."""
        obs = self.obs_source()
        if not (obs.intruded or self._pause_requested):
            return False

        reason = "사람 감지" if obs.intruded else "일시정지"
        self.robot.estop()
        self._set_state(State.PAUSED, reason)

        while not self._stop:
            obs = self.obs_source()
            if not obs.intruded and not self._pause_requested:
                break
            time.sleep(0.05)
        if self._stop:
            return True

        self.robot.release_estop()
        # 재개할 때 이미 한 점은 그대로 둔다 — 다시 납땜하면 부품이 상한다.
        # (공 분류에서는 사람이 공을 옮겼을 수 있어 목록을 비웠지만, 여기서는
        #  '이미 납땜함' 이 물리적으로 되돌릴 수 없는 사실이다.)
        self._set_state(State.SCAN, "재개")
        if self.stats is not None:
            self.stats.record_pause()
        return True

    def _wait_for_latency(self) -> bool:
        if self.watchdog is None:
            return False
        waited = False
        while not self._stop and not self.watchdog.may_start_motion():
            if not waited:
                self._set_state(State.PAUSED,
                                f"지연 {self.watchdog.status_text()} — 새 동작 보류")
                waited = True
            time.sleep(0.05)
        return waited

    # ── 한 점 납땜 ────────────────────────────────────────────────────────
    def _solder_one(self, point: Ball) -> bool:
        """한 점을 납땜한다. 끝까지 마쳤으면 True."""
        started = time.monotonic()
        px, py = self.mapper.to_robot(point.u, point.v)
        self._emit("target", color=point.color, pixel=point.uv, robot=(px, py))

        try:
            # ① 안전 높이로 **옆으로** 이동. 인두 끝을 보드에 끌지 않기 위해
            #    수평 이동은 반드시 이 높이에서만 한다.
            self._set_state(State.APPROACH, f"납땜점 ({px:+.3f}, {py:+.3f})")
            self.robot.move_to([px, py, self.conf.z_safe], down=True)

            # ② 수직 하강
            self._set_state(State.DESCEND)
            self.robot.move_to([px, py, self.conf.z_solder], down=True)

            # ③ 체류 — 중간에 끊길 수 있다
            self._set_state(State.DWELL, f"{self.conf.dwell_s:.1f}초 체류")
            completed = self._dwell(self.conf.dwell_s)

            # ④ 수직 상승. **중단됐어도 반드시 올린다** — 인두를 보드에 얹은 채
            #    두면 그게 제일 나쁘다. E-STOP 이 걸렸으면 그것마저 못 하므로
            #    그때는 사람이 처리해야 한다(정지 상태에서 팔을 움직이지 않는다).
            self._set_state(State.RETRACT)
            self.robot.move_to([px, py, self.conf.z_safe], down=True)
        except OutOfReach as exc:
            self._emit("skip", reason=f"작업영역 밖: {exc}")
            self._done.append((point.color, px, py))    # 다시 시도하지 않는다
            if self.stats is not None:
                self.stats.record_unreachable()
            return False

        if not completed or self.robot.estopped:
            self._emit("skip", reason="체류가 중단됨 — 이 점은 다시 시도한다")
            return False

        self._done.append((point.color, px, py))
        elapsed = time.monotonic() - started
        self._emit("soldered", color=point.color, seconds=elapsed,
                   index=len(self._done))
        if self.stats is not None:
            self.stats.record_success(point.color, elapsed)
        return True

    def _finish(self, count: int) -> None:
        """남은 점이 없다 — 안전 자세로 돌아간다."""
        self._emit("complete", count=count)
        try:
            self.robot.home()
        except OutOfReach:
            pass
        self._set_state(State.DONE, f"납땜 {count}점 완료")
