# -*- coding: utf-8 -*-
"""teach.py — 손으로 가르쳐서 값을 잰다. 추정값을 코드에 박지 않기 위한 도구.

토크를 끄면 팔을 손으로 움직일 수 있다. 그 상태에서 관절각을 읽어 FK 로 풀면
"지금 손끝이 어디 있는지"를 로봇이 스스로 알려준다. 자를 대지 않아도 되고,
로봇 좌표계 그대로 나오므로 변환 오차도 없다.

재는 것들:

    limits    작업영역 경계 — 안전한 범위를 손으로 한 바퀴 훑는다
    heights   z_grasp / z_hover / z_release — 책상·bin 에 팔끝을 대고 읽는다
    grip      그리퍼 열림·닫힘·빈손 판정 기준 — 공을 물려보고 읽는다
    home      안전 대기 자세
    add       이름 붙인 좌표 (bin 위치 등)

**하드웨어가 말을 안 들을 때**: 서보가 응답하지 않으면 자동 읽기가 실패한다.
그때 막히지 않도록 모든 항목에 수동 입력 경로가 있다. 자로 잰 값을 넣고
진행하되, 수동이었다는 사실과 사유는 파일에 남는다.

    python -m sorting.teach limits
    python -m sorting.teach heights
    python -m sorting.teach grip
    python -m sorting.teach add bin_red
    python -m sorting.teach list
"""
from __future__ import annotations

import time
from dataclasses import dataclass

from . import calib
from . import config as cfg

POSES_KEY = "poses"
TRACE_HZ = 10.0          # 경계를 훑는 동안 초당 몇 점을 기록할지


@dataclass(frozen=True)
class Pose:
    """이름 붙인 손끝 좌표. 관절각도 같이 남겨 재현할 수 있게 한다."""
    name: str
    xyz: tuple[float, float, float]
    joints_deg: tuple[float, ...] = ()

    def to_dict(self) -> dict:
        return {"xyz": list(self.xyz), "joints_deg": list(self.joints_deg)}

    @classmethod
    def from_dict(cls, name: str, d: dict) -> "Pose":
        return cls(name, tuple(d["xyz"]), tuple(d.get("joints_deg", ())))


class PoseLibrary:
    """티칭한 좌표들. 실측값 저장소에 함께 들어간다."""

    def __init__(self):
        self._poses: dict[str, Pose] = {}
        self.load()

    def load(self) -> None:
        raw = calib.STORE.get(POSES_KEY, {}).value or {}
        self._poses = {n: Pose.from_dict(n, d) for n, d in raw.items()
                       if isinstance(d, dict) and "xyz" in d}

    def save(self, manual: bool = False, note: str = "") -> None:
        payload = {n: p.to_dict() for n, p in self._poses.items()}
        setter = calib.STORE.set_manual if manual else calib.STORE.set_measured
        setter(POSES_KEY, payload, note)

    def add(self, pose: Pose, manual: bool = False, note: str = "") -> Pose:
        self._poses[pose.name] = pose
        self.save(manual=manual, note=note)
        return pose

    def get(self, name: str) -> Pose | None:
        return self._poses.get(name)

    def remove(self, name: str) -> bool:
        gone = self._poses.pop(name, None) is not None
        if gone:
            self.save()
        return gone

    def names(self) -> list[str]:
        return sorted(self._poses)

    def __len__(self) -> int:
        return len(self._poses)


class ReadFailed(RuntimeError):
    """서보에서 자세를 읽지 못했다. 부르는 쪽이 수동 경로를 제안해야 한다."""


class TeachSession:
    """토크를 끄고 손끝 좌표를 읽는 한 번의 티칭 작업.

    `reader` 를 주입할 수 있어 하드웨어 없이 테스트된다. 실제로는 서보에서
    관절각을 읽어 FK 로 손끝 위치를 푸는 함수가 들어간다.
    """

    def __init__(self, reader=None, port: str | None = None):
        self._reader = reader
        self._drv = None
        self.port = port or cfg.ROBOT.port

    # ── 하드웨어 ──────────────────────────────────────────────────────────
    def _ensure_hardware(self):
        if self._reader is not None:
            return
        from soarm_lab.driver_sdk import STS3215Driver
        from soarm_lab.fk_core import FKSo101

        drv = STS3215Driver(port=self.port)
        drv.connect()
        fk = FKSo101()
        self._drv = drv

        def read():
            pos = drv.get_all_positions()
            deg = []
            for sid in (1, 2, 3, 4, 5):
                raw = pos.get(sid)
                if raw is None:
                    raise ReadFailed(f"서보 {sid} 가 응답하지 않습니다")
                deg.append(STS3215Driver.position_to_degrees(raw) or 0.0)
            p, _ = fk.fk_deg(deg)
            return (float(p[0]), float(p[1]), float(p[2])), tuple(deg)

        self._reader = read

    def relax(self) -> None:
        """토크를 꺼서 손으로 팔을 움직일 수 있게 한다."""
        self._ensure_hardware()
        if self._drv is not None:
            for sid in (1, 2, 3, 4, 5):
                self._drv.set_torque(sid, False)

    def hold(self) -> None:
        """토크를 다시 켠다. 티칭이 끝나면 팔이 늘어져 있지 않도록."""
        if self._drv is not None:
            for sid in (1, 2, 3, 4, 5):
                self._drv.set_torque(sid, True)

    def read_tip(self) -> tuple[tuple[float, float, float], tuple[float, ...]]:
        """지금 손끝 좌표와 관절각. 못 읽으면 ReadFailed."""
        self._ensure_hardware()
        return self._reader()

    def grip_percent(self) -> float | None:
        """현재 그리퍼 개도(%). 못 읽으면 None."""
        if self._drv is None:
            return None
        from .robot import GRIP_ID, GripMap
        raw = self._drv.get_position(GRIP_ID)
        return None if raw is None else GripMap.from_driver().raw_to_pct(raw)

    # ── 경계 훑기 ─────────────────────────────────────────────────────────
    def trace_boundary(self, seconds: float | None = None, stop_check=None,
                       hz: float = TRACE_HZ,
                       give_up_after_s: float = 3.0) -> list[tuple]:
        """테두리를 도는 동안 손끝 좌표를 계속 기록한다.

        seconds 를 주면 그 시간 동안, 아니면 stop_check() 가 True 를 돌려줄
        때까지 기록한다. 못 읽은 프레임은 건너뛴다 — 한 점 놓쳤다고 전체
        측정을 버릴 이유는 없다.
        """
        points: list[tuple] = []
        period = 1.0 / hz
        started = time.monotonic()
        misses = 0

        while True:
            if seconds is not None and time.monotonic() - started >= seconds:
                break
            if stop_check is not None and stop_check():
                break
            try:
                xyz, _deg = self.read_tip()
                points.append(xyz)
            except ReadFailed:
                misses += 1
                if misses > hz * give_up_after_s:
                    raise
            time.sleep(period)
        return points


# ── 대화형 명령들 ──────────────────────────────────────────────────────────
def _ask(prompt: str) -> str:
    try:
        return input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        return "q"


def _manual_float(label: str, note_hint: str) -> tuple[float, str] | None:
    """자동 읽기가 실패했을 때 손으로 값을 넣는 경로."""
    print(f"\n  자동 측정이 실패했습니다. {label} 을(를) 직접 넣을 수 있습니다.")
    print("  (자로 재거나 조깅으로 확인한 값. 취소하려면 그냥 Enter)")
    text = _ask(f"  {label} = ")
    if not text or text == "q":
        return None
    try:
        value = float(text)
    except ValueError:
        print("  숫자가 아닙니다 — 취소합니다.")
        return None
    reason = _ask("  왜 수동인가요? (기록에 남습니다) > ") or note_hint
    return value, reason


def cmd_limits(session: TeachSession) -> int:
    """작업영역 경계를 손으로 훑어 측정한다."""
    from .workspace import Workspace

    print("\n작업영역 측정 — 팔이 들어가도 되는 범위의 테두리를 훑습니다.")
    print("토크를 끄면 팔을 손으로 움직일 수 있습니다.")
    try:
        session.relax()
    except Exception as exc:
        print(f"\n토크를 끄지 못했습니다: {exc}")
        print("수동 경로: 반경·높이를 직접 넣어 사각 영역을 만듭니다.")
        return _limits_manual()

    print("\n  Enter → 기록 시작, 테두리를 천천히 한 바퀴, 다시 Enter → 종료")
    if _ask("  준비되면 Enter > ") == "q":
        return 1

    print(f"  기록 중… ({TRACE_HZ:.0f}Hz) 다 돌았으면 Enter")
    done = {"stop": False}

    import threading
    threading.Thread(target=lambda: (input(), done.__setitem__("stop", True)),
                     daemon=True).start()
    try:
        points = session.trace_boundary(stop_check=lambda: done["stop"])
    except ReadFailed as exc:
        print(f"\n  서보를 계속 읽지 못했습니다: {exc}")
        return _limits_manual()
    finally:
        session.hold()

    if len(points) < 3:
        print(f"  기록된 점이 {len(points)}개뿐입니다 — 더 천천히 돌아주세요.")
        return 1

    ws = Workspace.from_trace(points)
    ws.save(note=f"손으로 훑음 ({len(points)}점)")
    print(f"\n  ✅ 측정 완료 — {ws.summary}")
    print(f"     기록 {len(points)}점 → 경계 꼭짓점 {len(ws.polygon_xy)}개")
    return 0


def _limits_manual() -> int:
    """토크오프 추적이 안 될 때: 반경·높이를 직접 넣어 영역을 만든다."""
    import math

    from .workspace import Workspace

    print("\n  손으로 잰 값을 넣어 주세요 (취소는 Enter).")
    r_max = _manual_float("최대 반경(m, 예 0.28)", "토크오프 추적 실패")
    if r_max is None:
        return 1
    r_min = _manual_float("최소 반경(m, 예 0.12)", "토크오프 추적 실패")
    z_lo = _manual_float("최저 높이(m, 예 0.0)", "토크오프 추적 실패")
    z_hi = _manual_float("최고 높이(m, 예 0.22)", "토크오프 추적 실패")
    if None in (r_min, z_lo, z_hi):
        return 1

    # 원형 영역을 12각형으로 근사한다 — 손으로 넣은 값이라 정밀할 필요가 없다
    trace = []
    for i in range(12):
        a = 2 * math.pi * i / 12
        trace.append((r_max[0] * math.cos(a), r_max[0] * math.sin(a), z_lo[0]))
    trace.append((r_min[0], 0.0, z_hi[0]))

    ws = Workspace.from_trace(trace)
    ws.save(manual=True, note=f"수동 입력 — {r_max[1]}")
    print(f"\n  ✏️ 수동 등록 — {ws.summary}")
    print("     (나중에 토크오프가 되면 python -m sorting.teach limits 로 다시 재세요)")
    return 0


def cmd_heights(session: TeachSession) -> int:
    """z_grasp / z_hover / z_release 를 팔끝을 대어 측정한다."""
    steps = [
        ("robot.z_grasp", "공을 집을 높이 — 팔끝을 **책상 바닥**에 살짝 닿게"),
        ("robot.z_hover", "옮길 때 높이 — 공을 넘어갈 만큼 **위로** 들어서"),
        ("robot.z_release", "놓을 높이 — **bin 바닥 조금 위**에"),
    ]
    print("\n높이 측정 — 각 자세로 팔을 옮긴 뒤 Enter 를 누르세요.")
    try:
        session.relax()
    except Exception as exc:
        print(f"토크를 끄지 못했습니다: {exc}")

    for key, what in steps:
        print(f"\n  [{key}] {what}")
        if _ask("  자세를 잡고 Enter (건너뛰려면 s) > ") == "s":
            continue
        try:
            xyz, _deg = session.read_tip()
            calib.STORE.set_measured(key, round(float(xyz[2]), 4), "팔끝을 대고 FK 로 읽음")
            print(f"    ✅ z = {xyz[2]:.4f} m")
        except ReadFailed as exc:
            print(f"    읽기 실패: {exc}")
            got = _manual_float(f"{key} (m)", "서보 읽기 실패")
            if got:
                calib.STORE.set_manual(key, got[0], got[1])
                print(f"    ✏️ 수동 등록: {got[0]} m")
    session.hold()
    return 0


def cmd_grip(session: TeachSession) -> int:
    """그리퍼 열림·닫힘·빈손 판정 기준을 실제로 물려보고 측정한다."""
    print("\n그리퍼 측정 — 조깅이나 손으로 그리퍼를 움직여 각 상태를 만드세요.")
    steps = [
        ("robot.grip_open", "**활짝 열린** 상태"),
        ("robot.grip_closed", "**공을 물고 있는** 상태 (공을 끼워 주세요)"),
        ("robot.grip_empty_pct", "**빈손으로 완전히 닫은** 상태"),
    ]
    for key, what in steps:
        print(f"\n  [{key}] {what}")
        if _ask("  그 상태로 만들고 Enter (건너뛰려면 s) > ") == "s":
            continue
        pct = None
        try:
            pct = session.grip_percent()
        except Exception as exc:
            print(f"    읽기 오류: {exc}")

        if pct is None:
            print("    그리퍼 위치를 읽지 못했습니다(서보6 응답 없음).")
            got = _manual_float(f"{key} (%)", "그리퍼 위치 읽기 실패")
            if got:
                calib.STORE.set_manual(key, got[0], got[1])
                print(f"    ✏️ 수동 등록: {got[0]}%")
            continue

        value = round(pct, 1)
        if key == "robot.grip_empty_pct":
            # 빈손일 때보다 조금 위를 기준으로 잡는다. 공을 물면 그 두께만큼
            # 덜 닫히므로, 빈손 값 그대로 쓰면 경계에서 오판이 난다.
            value = round(pct + 3.0, 1)
            print(f"    측정 {pct:.1f}% → 여유 3%p 를 더해 기준 {value}% 로 저장")
        calib.STORE.set_measured(key, value, "실제 그리퍼 상태에서 읽음")
        print(f"    ✅ {value}%")
    return 0


def cmd_home(session: TeachSession) -> int:
    """안전 대기 자세를 관절각으로 기록한다."""
    print("\n홈 자세 측정 — 카메라·bin 에 걸리지 않는 대기 자세를 잡아 주세요.")
    try:
        session.relax()
    except Exception as exc:
        print(f"토크를 끄지 못했습니다: {exc}")
    if _ask("  자세를 잡고 Enter > ") == "q":
        return 1
    try:
        _xyz, deg = session.read_tip()
        calib.STORE.set_measured("robot.home_pose_deg",
                                 [round(d, 1) for d in deg], "손으로 잡은 자세")
        print(f"  ✅ {[round(d, 1) for d in deg]}")
    except ReadFailed as exc:
        print(f"  읽기 실패: {exc} — 수동으로 넣으려면:")
        print("    python -m sorting.calib set robot.home_pose_deg '[0,30,-45,0,0]'"
              " --note '서보 읽기 실패'")
        return 1
    finally:
        session.hold()
    return 0


def cmd_add(session: TeachSession, name: str) -> int:
    """이름 붙인 좌표를 기록한다 (bin 위치 등)."""
    library = PoseLibrary()
    print(f"\n'{name}' 위치 측정 — 팔끝을 그 지점에 대어 주세요.")
    try:
        session.relax()
    except Exception as exc:
        print(f"토크를 끄지 못했습니다: {exc}")
    if _ask("  Enter 로 확정 > ") == "q":
        return 1
    try:
        xyz, deg = session.read_tip()
        library.add(Pose(name, xyz, deg), note=f"'{name}' 티칭")
        print(f"  ✅ {name} = ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f})")
    except ReadFailed as exc:
        print(f"  읽기 실패: {exc}")
        print("  수동 입력: x y z 를 공백으로 구분해 넣어 주세요 (취소는 Enter)")
        text = _ask("  x y z = ")
        parts = text.split()
        if len(parts) != 3:
            return 1
        xyz = tuple(float(v) for v in parts)
        reason = _ask("  왜 수동인가요? > ") or "서보 읽기 실패"
        library.add(Pose(name, xyz), manual=True, note=reason)
        print(f"  ✏️ 수동 등록: {name} = {xyz}")
    finally:
        session.hold()
    return 0


def cmd_list() -> int:
    library = PoseLibrary()
    if not len(library):
        print("등록된 좌표가 없습니다.  python -m sorting.teach add <이름>")
        return 0
    src = calib.STORE.get(POSES_KEY, None)
    print(f"\n등록된 좌표 {len(library)}개  [{src.badge}]")
    for name in library.names():
        p = library.get(name)
        print(f"  {name:<16} ({p.xyz[0]:+.3f}, {p.xyz[1]:+.3f}, {p.xyz[2]:+.3f})")
    return 0


def main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    ap = argparse.ArgumentParser(description="손으로 가르쳐서 값을 잰다")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("limits", help="작업영역 경계를 훑어서 측정")
    sub.add_parser("heights", help="파지·이동·놓기 높이 측정")
    sub.add_parser("grip", help="그리퍼 열림/닫힘/빈손 기준 측정")
    sub.add_parser("home", help="안전 대기 자세 측정")
    sub.add_parser("list", help="등록된 좌표 보기")
    p_add = sub.add_parser("add", help="이름 붙인 좌표 등록")
    p_add.add_argument("name")
    p_rm = sub.add_parser("remove", help="좌표 삭제")
    p_rm.add_argument("name")

    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    if args.cmd == "list":
        return cmd_list()
    if args.cmd == "remove":
        print("삭제됨:" if PoseLibrary().remove(args.name) else "그런 좌표 없음:",
              args.name)
        return 0

    session = TeachSession()
    handlers = {"limits": cmd_limits, "heights": cmd_heights,
                "grip": cmd_grip, "home": cmd_home}
    if args.cmd == "add":
        return cmd_add(session, args.name)
    return handlers[args.cmd](session)


if __name__ == "__main__":
    raise SystemExit(main())
