# -*- coding: utf-8 -*-
"""tunes.py — 부저로 울릴 곡. 순수 데이터라 여기만 고치면 소리가 바뀐다.

표기법:  "음이름옥타브-길이"  를 공백으로 나열한다.
    C4-4     4분음표 도(4옥타브)
    A#5-8    8분음표 라#(5옥타브)   — b(플랫)도 쓸 수 있다: Bb5-8
    R-16     16분쉼표
    G4-8.    점8분음표(길이 1.5배)

lead 는 D9 핀 부저, bass 는 D10 핀 부저로 나간다. 두 성부가 **동시에** 울린다
(Uno 내장 tone() 은 한 번에 한 음뿐이라, 스케치가 Timer1/Timer2 를 따로 쓴다).
두 성부의 길이 합이 달라도 되며, 짧은 쪽이 먼저 끝나고 조용해진다.

    python -m sorting.sound --test victory     # 로봇 없이 들어보기
"""
from __future__ import annotations

# ── 별의 커비 스테이지 클리어 팡파레 ───────────────────────────────────────
#
# ⚠ 아래 음은 기억에 의존해 옮긴 초안이라 실제 곡과 다를 수 있습니다.
#   한 번 울려보고 귀에 맞게 이 두 줄만 고치면 됩니다 — 다른 파일은 건드릴
#   필요가 없습니다. (--test kirby 로 바로 들어볼 수 있습니다.)
#
KIRBY_CLEAR = {
    "lead": "C5-16 D5-16 E5-16 F5-16 G5-8 G5-16 A5-16 G5-8 E5-8 C6-4 R-8 G5-16 A5-16 C6-2",
    "bass": "C3-8 C3-8 G3-8 G3-8 C3-8 E3-8 G3-8 C4-2",
    "bpm": 150,
}

# 직접 쓴 대체 팡파레 — 위 초안이 마음에 안 들 때 VICTORY 를 이걸로 바꾸면 된다.
FANFARE = {
    "lead": "C5-8 E5-8 G5-8 C6-4 G5-8 C6-2",
    "bass": "C3-4 C3-4 G2-4 C3-2",
    "bpm": 140,
}

# ── 상황별 소리 ────────────────────────────────────────────────────────────
TUNES: dict[str, dict] = {
    # 전체 분류 완료 — 로봇 승리 동작·화면 배너와 함께 터진다
    "victory": KIRBY_CLEAR,
    "kirby": KIRBY_CLEAR,
    "fanfare": FANFARE,

    # 사람 손 감지 → 정지. 급하고 반복적인 저음이라 놓치기 어렵다.
    "warn": {"lead": "A4-16 R-16 A4-16 R-16 A4-16 R-16 A4-8",
             "bass": "A3-16 R-16 A3-16 R-16 A3-16 R-16 A3-8",
             "bpm": 200},

    # 손이 빠져 재개 — 올라가는 두 음으로 '다시 움직입니다'를 알린다
    "resume": {"lead": "C5-16 G5-8", "bass": "C4-16 E4-8", "bpm": 160},

    # 공 하나 성공 — 짧게 '띵'
    "pick_ok": {"lead": "E6-16", "bass": "", "bpm": 160},

    # 시작
    "start": {"lead": "G4-16 C5-16 E5-8", "bass": "C3-16 C3-16 C3-8", "bpm": 160},
}

DEFAULT_BPM = 140


# ── 음이름 → 주파수 ────────────────────────────────────────────────────────
_SEMITONE = {"C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9, "B": 11}


def note_to_hz(name: str) -> int:
    """'A4' → 440. 쉼표('R')는 0.

    A4=440Hz 를 기준으로 반음 하나당 2^(1/12) 배. MIDI 번호로 환산해서 푼다.
    """
    name = name.strip().upper()
    if not name or name == "R":
        return 0
    letter, rest = name[0], name[1:]
    if letter not in _SEMITONE:
        raise ValueError(f"모르는 음이름: {name}")
    semitone = _SEMITONE[letter]
    while rest and rest[0] in "#B":
        semitone += 1 if rest[0] == "#" else -1
        rest = rest[1:]
    if not rest:
        raise ValueError(f"옥타브가 없다: {name}")
    octave = int(rest)
    midi = 12 * (octave + 1) + semitone
    return int(round(440.0 * (2.0 ** ((midi - 69) / 12.0))))


def parse(voice: str, bpm: int = DEFAULT_BPM) -> list[tuple[int, int]]:
    """악보 문자열 → [(주파수Hz, 지속시간ms), ...]. 주파수 0 은 쉼표."""
    whole_ms = 4 * 60_000 / max(1, bpm)      # 온음표 = 4박
    out: list[tuple[int, int]] = []
    for token in voice.split():
        if "-" not in token:
            raise ValueError(f"길이가 없는 음: {token} (예: C5-8)")
        note, length = token.rsplit("-", 1)
        dotted = length.endswith(".")
        denom = int(length.rstrip("."))
        ms = whole_ms / denom * (1.5 if dotted else 1.0)
        out.append((note_to_hz(note), int(round(ms))))
    return out


def get(name: str) -> dict:
    """이름으로 곡을 꺼낸다. 없으면 KeyError 대신 알아듣기 쉬운 메시지."""
    if name not in TUNES:
        raise KeyError(f"모르는 곡 '{name}'. 있는 것: {', '.join(sorted(TUNES))}")
    return TUNES[name]


def duration_ms(name: str) -> int:
    """그 곡이 몇 ms 짜리인지(긴 성부 기준)."""
    tune = get(name)
    bpm = tune.get("bpm", DEFAULT_BPM)
    return max((sum(ms for _hz, ms in parse(tune.get(v, ""), bpm))
                for v in ("lead", "bass")), default=0)
