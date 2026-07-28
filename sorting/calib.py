# -*- coding: utf-8 -*-
"""calib.py — 실측값 저장소. "이 숫자는 어디서 왔나"에 항상 답할 수 있게 한다.

이 프로젝트에서 반복해서 문제가 됐던 것이 하나 있다. **잴 수 있는 값을 추정값으로
코드에 박아두는 것.** 주황 구역 색은 촬영샷에 없어서 눈대중으로 넣었고, 도달 한계는
아예 없어서 IK 잔차에 맡겼다가 반경 0.44m 까지 통과하는 걸 뒤늦게 알았다.
둘 다 "실측 가능했는데 안 잰" 경우다.

그래서 규칙을 하나 세운다.

    잴 수 있으면 재고, 잰 값은 파일에 남기고, 화면에는 출처를 같이 보여준다.

값마다 출처가 셋 중 하나로 붙는다.

    measured  자동 측정 도구가 잰 값        (가장 믿을 만함)
    manual    사람이 직접 넣은 값           (하드웨어 문제로 자동 측정이 안 될 때)
    default   아직 아무도 안 잰 config 기본값 (경고 대상)

manual 이 필요한 이유: 서보가 응답을 안 하거나 카메라가 지저분해도 데모는 해야 한다.
자동 측정이 막혔을 때 **진행을 멈추지 않고** 사람이 자로 잰 값을 넣을 수 있어야 한다.
대신 그 값이 수동이었다는 사실과 이유는 파일에 남는다.

    python -m sorting.calib report
    python -m sorting.calib set robot.z_grasp 0.008 --note "서보4 응답불량, 자로 측정"
    python -m sorting.calib unset vision.orange
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from typing import Any

from . import config as cfg

MEASURED = "measured"
MANUAL = "manual"
DEFAULT = "default"

_BADGE = {MEASURED: "✅ 측정", MANUAL: "✏️ 수동", DEFAULT: "⚠️ 기본"}

# 이 프로젝트에서 실측 가능한 값들. report 가 "아직 안 잰 것"을 알려주려면
# 무엇을 재야 하는지 목록이 있어야 한다. (키, 설명, 재는 방법)
MEASURABLE: list[tuple[str, str, str]] = [
    ("vision.red", "빨간 공 Lab 기준색", "python -m sorting.lab_sample"),
    ("vision.blue", "파란 공 Lab 기준색", "python -m sorting.lab_sample"),
    ("vision.orange", "주황 공급구역 Lab 기준색", "python -m sorting.lab_sample"),
    ("vision.min_area_ball", "공 최소 면적(px)", "python -m sorting.measure areas"),
    ("vision.max_area_ball", "공 최대 면적(px)", "python -m sorting.measure areas"),
    ("vision.min_area_region", "구역 최소 면적(px)", "python -m sorting.measure areas"),
    ("vision.min_circularity", "공 최소 원형도", "python -m sorting.measure areas"),
    ("robot.ball_radius", "공 반지름(m)", "python -m sorting.measure ball"),
    ("robot.z_grasp", "파지 높이(m)", "python -m sorting.teach heights"),
    ("robot.z_hover", "이동 높이(m)", "python -m sorting.teach heights"),
    ("robot.z_release", "놓는 높이(m)", "python -m sorting.teach heights"),
    ("robot.grip_open", "그리퍼 열림(0~1)", "python -m sorting.teach grip"),
    ("robot.grip_closed", "그리퍼 닫힘(0~1)", "python -m sorting.teach grip"),
    ("robot.grip_empty_frac", "빈손 판정 기준(0~1)", "python -m sorting.teach grip"),
    ("robot.home_pose_deg", "안전 대기 자세(도)", "python -m sorting.teach home"),
    ("safety.intrusion_mm", "침입 판정 높이(mm)", "python -m sorting.measure depth"),
    ("workspace", "작업영역 경계", "python -m sorting.teach_limits"),
]


@dataclass(frozen=True)
class Value:
    """값 하나와 그 출처."""
    value: Any
    source: str
    at: float = 0.0
    note: str = ""

    @property
    def badge(self) -> str:
        return _BADGE.get(self.source, self.source)

    @property
    def is_default(self) -> bool:
        return self.source == DEFAULT

    def age_text(self) -> str:
        if not self.at:
            return ""
        days = (time.time() - self.at) / 86400
        if days < 1:
            return f"{int(days * 24)}시간 전"
        return f"{int(days)}일 전"


class CalibStore:
    """`data/calibration.json` 하나에 실측값을 모은다.

    소비자는 `config` 를 직접 읽지 않고 이 저장소를 거친다. 그래야 값이
    바뀌었을 때 코드를 고치지 않아도 되고, 출처를 화면에 띄울 수 있다.
    """

    def __init__(self, path: str | None = None, autoload: bool = True):
        self.path = path or os.path.join(cfg.DATA, "calibration.json")
        self._items: dict[str, Value] = {}
        if autoload:
            self.load()

    # ── 읽기 ──────────────────────────────────────────────────────────────
    def get(self, key: str, default: Any) -> Value:
        """저장된 값이 있으면 그것을, 없으면 default 를 DEFAULT 출처로 돌려준다."""
        found = self._items.get(key)
        return found if found is not None else Value(default, DEFAULT)

    def value(self, key: str, default: Any) -> Any:
        """출처 없이 값만 필요할 때."""
        return self.get(key, default).value

    def __contains__(self, key: str) -> bool:
        return key in self._items

    # ── 쓰기 ──────────────────────────────────────────────────────────────
    def set_measured(self, key: str, value: Any, note: str = "") -> Value:
        return self._set(key, value, MEASURED, note)

    def set_manual(self, key: str, value: Any, note: str = "") -> Value:
        """자동 측정이 하드웨어 문제로 막혔을 때 사람이 직접 넣는 값.

        note 에 왜 수동이었는지를 남겨두면 나중에 "이 값 왜 이래?"에 답할 수 있다.
        """
        return self._set(key, value, MANUAL, note)

    def _set(self, key: str, value: Any, source: str, note: str) -> Value:
        item = Value(value, source, time.time(), note)
        self._items[key] = item
        self.save()
        return item

    def clear(self, key: str) -> bool:
        """기본값으로 되돌린다. 잘못 잰 값을 빼는 용도."""
        existed = self._items.pop(key, None) is not None
        if existed:
            self.save()
        return existed

    # ── 파일 ──────────────────────────────────────────────────────────────
    def load(self) -> None:
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path, encoding="utf-8") as fh:
                raw = json.load(fh)
        except Exception as exc:
            # 파일이 깨졌다고 프로그램이 죽으면 안 된다 — 기본값으로 계속 간다
            print(f"[calib] 읽기 실패({self.path}): {exc} — 기본값으로 진행합니다")
            return
        for key, item in (raw.get("values") or {}).items():
            if isinstance(item, dict) and "value" in item:
                self._items[key] = Value(item["value"],
                                         item.get("source", MEASURED),
                                         float(item.get("at", 0.0)),
                                         item.get("note", ""))

    def save(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        payload = {
            "_comment": "실측·수동 입력값. 설치마다 다르므로 git 에 올리지 않는다.",
            "saved_at": time.time(),
            "values": {k: {"value": v.value, "source": v.source,
                           "at": v.at, "note": v.note}
                       for k, v in self._items.items()},
        }
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
        os.replace(tmp, self.path)      # 쓰다 죽어도 기존 파일이 남게

    # ── 현황 ──────────────────────────────────────────────────────────────
    def report(self) -> list[tuple[str, str, Value]]:
        """(키, 설명, 값) 목록. 아직 안 잰 것도 DEFAULT 로 함께 나온다."""
        rows = []
        for key, desc, _how in MEASURABLE:
            rows.append((key, desc, self.get(key, None)))
        for key, item in sorted(self._items.items()):
            if not any(key == k for k, _d, _h in MEASURABLE):
                rows.append((key, "(목록에 없는 값)", item))
        return rows

    def missing(self) -> list[tuple[str, str, str]]:
        """아직 측정도 수동입력도 안 된 항목. 데모 전에 확인용."""
        return [(k, d, how) for k, d, how in MEASURABLE if k not in self._items]


# 프로세스 전역 인스턴스. 다른 모듈은 이걸 가져다 쓴다.
STORE = CalibStore()


def get(key: str, default: Any) -> Any:
    """`calib.get("robot.z_grasp", cfg.ROBOT.z_grasp)` 형태로 쓴다."""
    return STORE.value(key, default)


def source_of(key: str) -> str:
    return STORE.get(key, None).source


# ── CLI ────────────────────────────────────────────────────────────────────
def _parse_literal(text: str) -> Any:
    """CLI 로 받은 문자열을 숫자/리스트/문자열로 해석한다."""
    import ast
    try:
        return ast.literal_eval(text)
    except (ValueError, SyntaxError):
        return text


def _cmd_report(store: CalibStore) -> int:
    rows = store.report()
    width = max(len(k) for k, _d, _v in rows) + 2
    print(f"\n실측값 현황  ({store.path})\n")
    for key, desc, item in rows:
        if item.value is None and item.is_default:
            print(f"  {'⚠️ 기본':<7} {key:<{width}} —      {desc}")
        else:
            age = f"({item.age_text()})" if item.at else ""
            note = f"  ← {item.note}" if item.note else ""
            print(f"  {item.badge:<7} {key:<{width}} {item.value!r:<22} {age}{note}")

    gaps = store.missing()
    if gaps:
        print(f"\n아직 안 잰 값 {len(gaps)}개 — 재는 방법:")
        for key, desc, how in gaps:
            print(f"    {key:<26} {desc:<22} {how}")
    else:
        print("\n모든 항목이 측정 또는 수동으로 채워져 있습니다.")
    return 0


def main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    ap = argparse.ArgumentParser(description="실측값 저장소 조회·수정")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("report", help="무엇이 측정/수동/기본인지 보기")

    p_set = sub.add_parser("set", help="값을 수동으로 넣기(자동 측정이 막혔을 때)")
    p_set.add_argument("key")
    p_set.add_argument("value")
    p_set.add_argument("--note", default="", help="왜 수동으로 넣는지")

    p_unset = sub.add_parser("unset", help="기본값으로 되돌리기")
    p_unset.add_argument("key")

    args = ap.parse_args(argv if argv is not None else sys.argv[1:])
    store = CalibStore()

    if args.cmd == "set":
        item = store.set_manual(args.key, _parse_literal(args.value), args.note)
        print(f"{item.badge}  {args.key} = {item.value!r}")
        if args.note:
            print(f"        사유: {args.note}")
        return 0
    if args.cmd == "unset":
        print("되돌림:" if store.clear(args.key) else "저장된 값 없음:", args.key)
        return 0
    return _cmd_report(store)


if __name__ == "__main__":
    raise SystemExit(main())
