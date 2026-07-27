# -*- coding: utf-8 -*-
"""악보 파싱과 음정 계산을 검증한다.

여기서 제일 중요한 건 마지막 테스트다: 파이썬(sorting/tunes.py)과 아두이노
(firmware/buzzer/buzzer.ino)가 음이름을 각자 따로 주파수로 바꾼다. 둘이
어긋나면 PC 스피커로 맞춰놓은 곡이 부저에서는 다른 음으로 나온다.
"""
from __future__ import annotations

import math
import os
import re

import pytest

from sorting import tunes

FIRMWARE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "firmware", "buzzer", "buzzer.ino")


def test_reference_pitch():
    """A4 = 440Hz 라는 기준점."""
    assert tunes.note_to_hz("A4") == 440


@pytest.mark.parametrize("name,hz", [
    ("C4", 262), ("E4", 330), ("G4", 392), ("C5", 523), ("A5", 880), ("C6", 1047),
])
def test_known_pitches(name, hz):
    """널리 쓰이는 음들이 표준 주파수와 1Hz 안에서 맞는가."""
    assert abs(tunes.note_to_hz(name) - hz) <= 1


def test_octave_doubles_frequency():
    for name in ("C4", "E4", "G4"):
        low = tunes.note_to_hz(name)
        high = tunes.note_to_hz(name[0] + str(int(name[1]) + 1))
        assert abs(high - 2 * low) <= 1


def test_accidentals():
    assert tunes.note_to_hz("A#4") == tunes.note_to_hz("Bb4")
    assert tunes.note_to_hz("C#4") > tunes.note_to_hz("C4")


def test_rest_is_silent():
    assert tunes.note_to_hz("R") == 0
    assert tunes.parse("R-4")[0][0] == 0


def test_bad_note_raises():
    for bad in ("H4", "C", "X#2"):
        with pytest.raises(ValueError):
            tunes.note_to_hz(bad)
    with pytest.raises(ValueError):
        tunes.parse("C5")          # 길이가 없다


def test_note_lengths():
    """4분음표 하나는 120bpm 에서 500ms."""
    (_hz, ms), = tunes.parse("C5-4", bpm=120)
    assert ms == 500
    (_hz, half), = tunes.parse("C5-2", bpm=120)
    assert half == 1000
    (_hz, dotted), = tunes.parse("C5-4.", bpm=120)
    assert dotted == 750


def test_all_tunes_parse():
    """등록된 모든 곡이 문법에 맞고, 길이가 상식적인가."""
    for name in tunes.TUNES:
        tune = tunes.get(name)
        bpm = tune.get("bpm", tunes.DEFAULT_BPM)
        for voice in ("lead", "bass"):
            tunes.parse(tune.get(voice, ""), bpm)
        ms = tunes.duration_ms(name)
        assert 0 < ms < 15000, f"{name} 길이가 이상하다: {ms}ms"


def test_warning_is_short_enough_to_be_urgent():
    """경고음은 짧아야 한다 — 손이 들어왔는데 3초짜리 곡이 흐르면 곤란하다."""
    assert tunes.duration_ms("warn") <= 1500
    assert tunes.duration_ms("resume") <= 1000
    assert tunes.duration_ms("pick_ok") <= 500


def test_unknown_tune_names_the_alternatives():
    with pytest.raises(KeyError) as exc:
        tunes.get("없는곡")
    assert "victory" in str(exc.value), "무엇을 쓸 수 있는지 알려줘야 한다"


# ── 파이썬 ↔ 아두이노 일치 검증 ────────────────────────────────────────────
def _arduino_c8_table() -> list[int]:
    """buzzer.ino 의 C8_TABLE 상수를 그대로 읽어온다."""
    src = open(FIRMWARE, encoding="utf-8").read()
    match = re.search(r"C8_TABLE\[12\]\s*=\s*\{([^}]+)\}", src)
    assert match, "buzzer.ino 에서 C8_TABLE 을 못 찾았다"
    return [int(x.strip()) for x in match.group(1).split(",")]


def test_firmware_pitch_table_matches_python():
    """아두이노의 음정표가 파이썬 계산과 같은가.

    펌웨어는 부동소수 pow() 를 피하려고 8옥타브 주파수를 표로 두고 옥타브마다
    2로 나눈다. 그 표가 틀리면 부저만 음이 어긋나는데, 소리로는 원인을 찾기
    어렵다 — 그래서 여기서 잡는다.
    """
    table = _arduino_c8_table()
    names = ["C8", "C#8", "D8", "D#8", "E8", "F8",
             "F#8", "G8", "G#8", "A8", "A#8", "B8"]
    for name, firmware_hz in zip(names, table):
        expected = tunes.note_to_hz(name)
        assert abs(expected - firmware_hz) <= 2, \
            f"{name}: 펌웨어 {firmware_hz}Hz vs 파이썬 {expected}Hz"


def _cents(hz: float, reference: float) -> float:
    """두 주파수의 음악적 거리(센트). 반음 = 100센트."""
    return abs(1200.0 * math.log2(hz / reference))


def test_both_implementations_stay_in_tune():
    """파이썬과 펌웨어가 각각 '진짜 음정'에 얼마나 가까운가.

    둘의 정수 결과를 직접 비교하면 안 된다 — 참값이 92.4986Hz 처럼 .5 근처면
    한쪽은 92, 한쪽은 93 으로 갈리고 그게 1% 차이로 보이지만 실제로는 둘 다
    0.5Hz 안에 있다. 그래서 음악적 단위(센트)로 참값과 견준다.
    25센트(1/4 반음)면 사람이 '다른 음'으로 듣지 않는다.
    """
    table = _arduino_c8_table()
    semitones = {"C": 0, "C#": 1, "D": 2, "D#": 3, "E": 4, "F": 5,
                 "F#": 6, "G": 7, "G#": 8, "A": 9, "A#": 10, "B": 11}

    for letter, idx in semitones.items():
        for octave in range(2, 9):
            midi = 12 * (octave + 1) + idx
            exact = 440.0 * 2.0 ** ((midi - 69) / 12.0)

            shift = 8 - octave
            base = table[idx]
            # buzzer.ino 의 noteToHz 와 같은 반올림 시프트
            firmware_hz = base if shift == 0 else (base + (1 << (shift - 1))) >> shift
            python_hz = tunes.note_to_hz(f"{letter}{octave}")

            assert _cents(firmware_hz, exact) < 25, \
                f"{letter}{octave}: 펌웨어 {firmware_hz}Hz 가 참값 {exact:.1f}Hz 에서 벗어남"
            assert _cents(python_hz, exact) < 25, \
                f"{letter}{octave}: 파이썬 {python_hz}Hz 가 참값 {exact:.1f}Hz 에서 벗어남"


def test_implementations_agree_where_it_matters():
    """실제 곡에 쓰인 음들은 두 구현이 사실상 같은 소리를 낸다."""
    table = _arduino_c8_table()
    semitones = {"C": 0, "C#": 1, "D": 2, "D#": 3, "E": 4, "F": 5,
                 "F#": 6, "G": 7, "G#": 8, "A": 9, "A#": 10, "B": 11}

    used: set[str] = set()
    for tune in tunes.TUNES.values():
        for voice in ("lead", "bass"):
            for token in tune.get(voice, "").split():
                note = token.rsplit("-", 1)[0].upper()
                if note != "R":
                    used.add(note)
    assert used, "곡에서 음을 하나도 못 읽었다"

    for note in sorted(used):
        letter, rest = note[0], note[1:]
        while rest and rest[0] in "#B":
            letter += "#" if rest[0] == "#" else "b"
            rest = rest[1:]
        octave = int(rest)
        idx = semitones[letter] if letter in semitones else None
        if idx is None:                       # 플랫 표기는 이 곡들에 없다
            continue
        shift = 8 - octave
        base = table[idx]
        firmware_hz = base if shift == 0 else (base + (1 << (shift - 1))) >> shift
        python_hz = tunes.note_to_hz(note)
        assert _cents(firmware_hz, python_hz) < 25, \
            f"{note}: 부저 {firmware_hz}Hz vs PC {python_hz}Hz — 곡이 다르게 들린다"


def test_firmware_declares_same_tune_names():
    """PC 가 보내는 이름을 보드가 전부 알아듣는가.

    파이썬에만 있는 곡을 PLAY 하면 보드가 'ERR unknown tune' 을 뱉고 조용히
    아무 소리도 안 난다 — 데모 중에 알아채기 힘든 종류의 실패다.
    """
    src = open(FIRMWARE, encoding="utf-8").read()
    declared = set(re.findall(r'NAME_\w+\[\]\s*PROGMEM\s*=\s*"([^"]+)"', src))
    missing = set(tunes.TUNES) - declared
    assert not missing, f"펌웨어에 없는 곡: {sorted(missing)}"
