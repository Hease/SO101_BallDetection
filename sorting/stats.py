# -*- coding: utf-8 -*-
"""stats.py — 사이클 통계. 화면에 실시간으로 띄우고 끝나면 파일로 남긴다.

세는 것: 색깔별 분류 개수, 사이클 소요시간, 파지 재시도·실패, 도달 불가,
사람 감지로 멈춘 횟수. 데모가 잘 됐는지를 인상이 아니라 숫자로 말할 수 있게 한다.
"""
from __future__ import annotations

import csv
import json
import os
import statistics
import time
from dataclasses import asdict, dataclass, field

from . import config as cfg


@dataclass
class Cycle:
    index: int
    color: str
    seconds: float
    at: float = field(default_factory=time.time)


class SessionStats:
    """한 번 실행하는 동안의 집계."""

    def __init__(self):
        self.started = time.time()
        self.cycles: list[Cycle] = []
        self.per_color: dict[str, int] = {}
        self.retries = 0
        self.failures: dict[str, int] = {}
        self.unreachable = 0
        self.pauses = 0

    # ── 기록 ──────────────────────────────────────────────────────────────
    def record_success(self, color: str, seconds: float) -> None:
        self.cycles.append(Cycle(len(self.cycles) + 1, color, seconds))
        self.per_color[color] = self.per_color.get(color, 0) + 1

    def record_retry(self) -> None:
        self.retries += 1

    def record_failure(self, color: str) -> None:
        self.failures[color] = self.failures.get(color, 0) + 1

    def record_unreachable(self) -> None:
        self.unreachable += 1

    def record_pause(self) -> None:
        self.pauses += 1

    # ── 조회 ──────────────────────────────────────────────────────────────
    @property
    def total(self) -> int:
        return len(self.cycles)

    @property
    def attempts(self) -> int:
        return self.total + sum(self.failures.values())

    @property
    def success_rate(self) -> float:
        """시도 대비 성공률(%). 시도가 없으면 100 으로 본다."""
        return 100.0 if self.attempts == 0 else 100.0 * self.total / self.attempts

    @property
    def mean_seconds(self) -> float:
        return statistics.fmean(c.seconds for c in self.cycles) if self.cycles else 0.0

    def summary_lines(self) -> list[str]:
        """GUI 패널에 그대로 뿌릴 수 있는 짧은 요약."""
        by_color = "  ".join(f"{c} {n}" for c, n in sorted(self.per_color.items()))
        return [
            f"분류 {self.total}개" + (f"   ({by_color})" if by_color else ""),
            f"성공률 {self.success_rate:.0f}%   평균 {self.mean_seconds:.1f}s",
            f"재시도 {self.retries}   실패 {sum(self.failures.values())}"
            f"   범위밖 {self.unreachable}   정지 {self.pauses}",
        ]

    # ── 저장 ──────────────────────────────────────────────────────────────
    def save(self, directory: str | None = None) -> tuple[str, str]:
        """JSON(전체)과 CSV(사이클 표)로 남긴다. 경로 두 개를 돌려준다."""
        directory = directory or cfg.STATS_DIR
        os.makedirs(directory, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.started))

        payload = {
            "started": self.started,
            "duration_s": time.time() - self.started,
            "total": self.total,
            "per_color": self.per_color,
            "success_rate": self.success_rate,
            "mean_seconds": self.mean_seconds,
            "retries": self.retries,
            "failures": self.failures,
            "unreachable": self.unreachable,
            "pauses": self.pauses,
            "cycles": [asdict(c) for c in self.cycles],
        }
        json_path = os.path.join(directory, f"session_{stamp}.json")
        with open(json_path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)

        csv_path = os.path.join(directory, f"session_{stamp}.csv")
        with open(csv_path, "w", encoding="utf-8", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["index", "color", "seconds", "at"])
            for c in self.cycles:
                writer.writerow([c.index, c.color, f"{c.seconds:.3f}", f"{c.at:.3f}"])

        return json_path, csv_path
