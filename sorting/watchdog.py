# -*- coding: utf-8 -*-
"""watchdog.py — 카메라·로봇이 느려졌을 때 시스템이 어떻게 버티는가.

지금까지 이 시스템은 지연을 **재기만** 했다(`fps`, `latency_ms` 를 화면에 띄우는
정도). 재는 것과 대응하는 것은 다르다. 카메라 프레임이 1초 늦게 들어오는데 그
1초 전 좌표로 팔을 보내면, 사람이 이미 손을 뻗은 자리로 가게 된다.

그래서 **단계적으로** 대응한다. 하나의 임계값으로 즉시 정지시키면 잠깐 끊긴
것에도 매번 서서 데모가 답답해지고, 반대로 아예 대응을 안 하면 위험하다.

    LEVEL 0 정상   경계 미만          그대로 진행
    LEVEL 1 경고   화면 경고 + 감속    아직 움직이되 조심해서
    LEVEL 2 보류   신규 동작 금지      진행 중인 동작만 마무리하고 대기
    LEVEL 3 정지   estop()            더는 못 믿는다

**임계값도 실측으로 정한다.** 이 파일의 기본값은 어디까지나 시작점이고,
`python -m sorting.watchdog measure` 로 정상 상태의 지연 분포를 재서
그 통계(중앙값·p95)에서 임계를 잡는 것이 옳다. 장비·해상도·USB 사정에 따라
정상 지연이 크게 다르기 때문이다.

기존 안전장치와 이어지는 부분: LEVEL 3 은 `robot.should_abort` 에 물린다.
`RobotController.wait_settled` 가 20ms 마다 그 훅을 확인하므로 **동작 한복판에서도**
선다 — 침입 감지가 쓰는 것과 똑같은 경로다.

    python -m sorting.watchdog --demo
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import IntEnum

from . import calib

CAMERA = "camera"
ROBOT = "robot"
LOOP = "loop"


class Level(IntEnum):
    NORMAL = 0
    WARN = 1
    HOLD = 2
    STOP = 3

    @property
    def label(self) -> str:
        return {0: "정상", 1: "경고", 2: "보류", 3: "정지"}[int(self)]


@dataclass
class Thresholds:
    """단계 경계(ms). 실측으로 덮어쓰는 것이 원칙 — 아래는 시작점일 뿐이다."""
    warn_ms: float = 300.0
    hold_ms: float = 800.0
    stop_ms: float = 1500.0

    # 악화는 즉시, 회복은 천천히. 깜빡임 한 번에 로봇이 섰다 갔다 하면
    # 그게 더 위험하고 보기에도 불안하다.
    recover_samples: int = 5

    @classmethod
    def from_calib(cls) -> "Thresholds":
        d = cls()
        return cls(
            warn_ms=float(calib.get("watchdog.warn_ms", d.warn_ms)),
            hold_ms=float(calib.get("watchdog.hold_ms", d.hold_ms)),
            stop_ms=float(calib.get("watchdog.stop_ms", d.stop_ms)),
            recover_samples=int(calib.get("watchdog.recover_samples", d.recover_samples)),
        )


@dataclass
class Channel:
    """감시 대상 하나(카메라/로봇/루프)의 최근 상태."""
    name: str
    last_ms: float = 0.0
    worst_ms: float = 0.0
    samples: int = 0
    updated_at: float = field(default_factory=time.monotonic)


class LatencyWatchdog:
    """지연을 보고받아 단계를 매긴다.

    **다른 모듈에 전혀 의존하지 않는다** — 타임스탬프만 받는 순수 로직이다.
    그래서 로봇도 카메라도 없이 테스트할 수 있고, 담당자가 다른 사람의 작업을
    기다리지 않고 혼자 만들 수 있다.
    """

    def __init__(self, thresholds: Thresholds | None = None):
        self.th = thresholds or Thresholds.from_calib()
        self.channels: dict[str, Channel] = {}
        self._level = Level.NORMAL
        self._good_streak = 0
        self._injected_ms = 0.0
        self.changed_at = time.monotonic()

    # ── 보고 ──────────────────────────────────────────────────────────────
    def report(self, kind: str, ms: float) -> Level:
        """`kind` 채널의 지연이 `ms` 였다고 알린다. 갱신된 단계를 돌려준다."""
        ch = self.channels.setdefault(kind, Channel(kind))
        ch.last_ms = float(ms)
        ch.worst_ms = max(ch.worst_ms, ch.last_ms)
        ch.samples += 1
        ch.updated_at = time.monotonic()
        return self._recompute()

    def report_frame_age(self, captured_at: float) -> Level:
        """카메라 프레임이 언제 찍힌 것인지로 나이를 계산해 보고한다."""
        return self.report(CAMERA, (time.monotonic() - captured_at) * 1000.0)

    # ── 결함 주입 (데모용) ────────────────────────────────────────────────
    def inject(self, ms: float) -> None:
        """인위적 지연. 데모에서 3단계를 눈으로 보여주기 위한 것.

        측정값에 더해지므로, 슬라이더를 올리면 실제로 느려진 것과 같은 경로를
        탄다. 화면에는 '지연 주입 중'을 크게 띄워 진짜 고장과 구분한다.
        """
        self._injected_ms = max(0.0, float(ms))

    @property
    def injected_ms(self) -> float:
        return self._injected_ms

    # ── 판정 ──────────────────────────────────────────────────────────────
    def _observed_ms(self) -> float:
        base = max((c.last_ms for c in self.channels.values()), default=0.0)
        return base + self._injected_ms

    def _level_for(self, ms: float) -> Level:
        if ms >= self.th.stop_ms:
            return Level.STOP
        if ms >= self.th.hold_ms:
            return Level.HOLD
        if ms >= self.th.warn_ms:
            return Level.WARN
        return Level.NORMAL

    def _recompute(self) -> Level:
        target = self._level_for(self._observed_ms())

        if target > self._level:
            # 악화는 즉시, 그리고 몇 단계든 한 번에 올라간다
            self._level = target
            self._good_streak = 0
            self.changed_at = time.monotonic()
        elif target < self._level:
            self._good_streak += 1
            if self._good_streak >= self.th.recover_samples:
                # 회복은 **한 단계씩만** 내려온다.
                # 목표 단계로 곧장 뛰면, 900→400→200ms 처럼 아직 나쁜 값이 섞인
                # 구간에서도 "3회 개선"으로 세어져 정지에서 정상으로 건너뛴다.
                # 한 칸씩 내려오면 그 사이 한 번이라도 나빠질 때 즉시 되돌아간다.
                self._level = Level(int(self._level) - 1)
                self._good_streak = 0
                self.changed_at = time.monotonic()
        else:
            self._good_streak = 0         # 같은 단계면 회복 카운트를 리셋
        return self._level

    # ── 조회 ──────────────────────────────────────────────────────────────
    @property
    def level(self) -> Level:
        return self._level

    @property
    def observed_ms(self) -> float:
        return self._observed_ms()

    def worst_channel(self) -> tuple[str, float]:
        """가장 느린 채널. 화면에 '무엇이 느린지'를 말해주기 위한 것."""
        if not self.channels:
            return ("-", 0.0)
        ch = max(self.channels.values(), key=lambda c: c.last_ms)
        return (ch.name, ch.last_ms + self._injected_ms)

    # ── 바깥이 쓰는 결정 ──────────────────────────────────────────────────
    def should_abort(self) -> bool:
        """`robot.should_abort` 에 그대로 연결한다 — 이동 중에도 20ms 마다 검사된다."""
        return self._level >= Level.STOP

    def may_start_motion(self) -> bool:
        """새 동작을 시작해도 되는가. HOLD 부터는 시작하지 않는다."""
        return self._level < Level.HOLD

    def speed_scale(self) -> float:
        """WARN 에서 감속 배율. 느려진 만큼 팔도 천천히 간다."""
        return {Level.NORMAL: 1.0, Level.WARN: 0.5,
                Level.HOLD: 0.0, Level.STOP: 0.0}[self._level]

    def status_text(self) -> str:
        name, ms = self.worst_channel()
        base = f"{self._level.label}  {name} {ms:.0f}ms"
        return base + f"  (주입 {self._injected_ms:.0f}ms)" if self._injected_ms else base

    def reset(self) -> None:
        self._level = Level.NORMAL
        self._good_streak = 0
        for ch in self.channels.values():
            ch.worst_ms = 0.0


# ── 임계값 실측 ────────────────────────────────────────────────────────────
def measure_thresholds(samples_ms: list[float], factor: float = 3.0) -> Thresholds:
    """정상 상태의 지연 분포에서 임계값을 정한다.

    p95 를 '정상의 상한'으로 보고 그 배수로 단계를 나눈다. 장비가 빠르면 임계도
    낮아지고 느리면 높아진다 — 숫자를 손으로 정하는 것보다 환경에 맞는다.
    """
    if not samples_ms:
        raise ValueError("측정 표본이 없습니다")
    ordered = sorted(samples_ms)
    p95 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]
    warn = max(50.0, p95 * factor)
    return Thresholds(warn_ms=warn, hold_ms=warn * 2.5, stop_ms=warn * 5.0)


def _demo() -> int:
    """지연을 점점 올렸다 내리며 단계 전이를 보여준다(하드웨어 없이)."""
    wd = LatencyWatchdog(Thresholds(warn_ms=300, hold_ms=800, stop_ms=1500,
                                    recover_samples=3))
    print("지연을 올렸다 내리며 단계가 어떻게 바뀌는지 봅니다.")
    print(f"임계: 경고 {wd.th.warn_ms:.0f} / 보류 {wd.th.hold_ms:.0f} / "
          f"정지 {wd.th.stop_ms:.0f} ms, 회복은 연속 {wd.th.recover_samples}회\n")
    print(f"{'지연(ms)':>9}  {'단계':<5} {'새 동작':<7} {'속도':<6} 비고")
    print("-" * 62)

    script = [50, 120, 350, 500, 900, 1200, 1800, 2000,
              900, 400, 200, 100, 80, 60, 50]
    prev = wd.level
    for ms in script:
        wd.report(CAMERA, ms)
        note = ""
        if wd.level != prev:
            note = f"← {prev.label} 에서 {wd.level.label} 으로"
            prev = wd.level
        elif wd.level > Level.NORMAL and ms < wd.th.warn_ms:
            note = "회복 대기 중(연속 관찰)"
        print(f"{ms:>9}  {wd.level.label:<5} "
              f"{'가능' if wd.may_start_motion() else '금지':<7} "
              f"{wd.speed_scale():<6.1f} {note}")

    print("\n올라갈 때는 즉시 반응하고, 내려올 때는 연속으로 좋아야 풀립니다 —")
    print("깜빡임 한 번에 로봇이 다시 출발하면 그게 더 위험하기 때문입니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(_demo())
