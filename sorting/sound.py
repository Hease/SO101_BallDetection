# -*- coding: utf-8 -*-
"""sound.py — 부저로 소리를 낸다. 로봇 제어 루프를 절대 붙잡지 않는다.

곡은 아두이노 안에 있고 PC 는 "PLAY:victory\\n" 한 줄만 던진다. 그래서 3초짜리
팡파레가 울리는 동안에도 파이썬 쪽은 즉시 다음 일을 한다. 게다가 시리얼 쓰기마저
따로 스레드에 맡겨, 아두이노가 응답하지 않아도 로봇이 멈추지 않는다.

부저가 없어도 전부 정상 동작한다 — 포트를 못 찾으면 조용한 NullBackend 로 내려간다.
그래서 하드웨어를 나중에 붙여도 코드는 그대로다.

    python -m sorting.sound --list
    python -m sorting.sound --test victory
    python -m sorting.sound --test victory --pc      # 부저 없이 PC 스피커로
"""
from __future__ import annotations

import glob
import queue
import sys
import threading
import time

from . import config as cfg
from . import tunes


class SoundBackend:
    """소리를 내는 방법 하나. play() 는 반드시 즉시 반환해야 한다."""

    def play(self, name: str) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...

    @property
    def describe(self) -> str:
        return type(self).__name__


class NullBackend(SoundBackend):
    """부저가 없을 때. 조용히 아무것도 안 하지만, 무엇을 울리려 했는지는 남긴다."""

    def __init__(self, reason: str = "부저 없음"):
        self.reason = reason
        self.played: list[str] = []

    def play(self, name: str) -> None:
        self.played.append(name)

    @property
    def describe(self) -> str:
        return f"무음 ({self.reason})"


class ArduinoBuzzer(SoundBackend):
    """USB 시리얼로 아두이노에 곡 이름만 보낸다.

    쓰기를 워커 스레드에 맡기는 이유: pyserial 의 write 는 버퍼가 차면 블로킹한다.
    보드가 리셋 중이거나 케이블이 빠지면 그 한 줄에 로봇 스레드가 붙잡힐 수 있다.
    """

    def __init__(self, port: str, baud: int = 115200):
        import serial
        self.port = port
        self.serial = serial.Serial(port, baud, timeout=0.5)
        time.sleep(2.0)              # Uno 는 포트를 열면 자동 리셋된다 — 부팅 대기
        self._q: queue.Queue[str | None] = queue.Queue(maxsize=8)
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self) -> None:
        while True:
            line = self._q.get()
            if line is None:
                return
            try:
                self.serial.write(line.encode("ascii"))
                self.serial.flush()
            except Exception as exc:          # 소리 때문에 로봇이 죽으면 안 된다
                print(f"[sound] 전송 실패({self.port}): {exc}")

    def _send(self, line: str) -> None:
        try:
            self._q.put_nowait(line)
        except queue.Full:
            pass                              # 밀렸으면 그냥 흘려보낸다

    def play(self, name: str) -> None:
        tunes.get(name)                       # 오타는 여기서 바로 걸린다
        self._send(f"PLAY:{name}\n")

    def stop(self) -> None:
        self._send("STOP\n")

    def close(self) -> None:
        self._q.put(None)
        try:
            self.serial.close()
        except Exception:
            pass

    @property
    def describe(self) -> str:
        return f"아두이노 부저 ({self.port})"


class PcSpeakerBackend(SoundBackend):
    """부저 없이 PC 스피커로 소리를 만든다(개발·테스트용).

    두 성부를 numpy 로 합성해 sounddevice 로 낸다. 실제 데모는 부저를 쓰지만,
    하드웨어를 조립하기 전에 곡을 고칠 때 이게 있으면 훨씬 빠르다.
    """

    def __init__(self):
        import numpy as np
        import sounddevice as sd
        self._np = np
        self._sd = sd
        self.rate = 44100

    def _render(self, name: str):
        np = self._np
        tune = tunes.get(name)
        bpm = tune.get("bpm", tunes.DEFAULT_BPM)
        voices = [tunes.parse(tune.get(v, ""), bpm) for v in ("lead", "bass")]
        total_ms = max((sum(ms for _f, ms in v) for v in voices), default=0)
        buf = np.zeros(int(self.rate * total_ms / 1000) + 1, dtype=np.float32)

        for voice, gain in zip(voices, (0.35, 0.22)):
            at = 0
            for hz, ms in voice:
                n = int(self.rate * ms / 1000)
                if hz > 0:
                    t = np.arange(n) / self.rate
                    wave = np.sign(np.sin(2 * np.pi * hz * t))     # 사각파 = 부저 음색
                    env = np.minimum(1.0, np.linspace(0, 12, n))   # 딱딱한 시작을 눌러준다
                    env *= np.minimum(1.0, np.linspace(12, 0, n))
                    buf[at:at + n] += (wave * env * gain).astype(np.float32)
                at += n
        return np.clip(buf, -1.0, 1.0)

    def play(self, name: str) -> None:
        self._sd.play(self._render(name), self.rate)

    def stop(self) -> None:
        self._sd.stop()

    @property
    def describe(self) -> str:
        return "PC 스피커"


# ── 백엔드 고르기 ──────────────────────────────────────────────────────────
def find_arduino_port(exclude: str | None = None) -> str | None:
    """아두이노로 보이는 포트를 찾는다. 로봇이 쓰는 포트는 반드시 피한다.

    로봇도 /dev/ttyACM* 로 잡히기 때문에, 여기서 실수하면 부저 명령이 서보
    드라이버로 날아간다 — 그래서 exclude 를 넘겨받아 명시적으로 제외한다.
    """
    exclude = exclude or cfg.ROBOT.port
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    for port in candidates:
        if port != exclude:
            return port
    return None


def make_backend(conf: cfg.SoundConfig | None = None,
                 prefer_pc: bool = False) -> SoundBackend:
    """설정에 맞는 백엔드 하나. 실패해도 예외 대신 무음으로 내려간다."""
    conf = conf or cfg.SOUND
    if not conf.enabled:
        return NullBackend("설정에서 꺼둠")

    if prefer_pc:
        try:
            return PcSpeakerBackend()
        except Exception as exc:
            return NullBackend(f"PC 스피커 사용 불가: {exc}")

    port = conf.port or find_arduino_port()
    if port is None:
        return NullBackend("아두이노 포트를 못 찾음")
    try:
        return ArduinoBuzzer(port, conf.baud)
    except Exception as exc:
        return NullBackend(f"{port} 열기 실패: {exc}")


# ── CLI ────────────────────────────────────────────────────────────────────
def _main(argv: list[str]) -> None:
    import argparse
    ap = argparse.ArgumentParser(description="부저 곡을 울려본다")
    ap.add_argument("--test", metavar="이름", help="울릴 곡 이름")
    ap.add_argument("--list", action="store_true", help="곡 목록")
    ap.add_argument("--pc", action="store_true", help="부저 대신 PC 스피커로")
    ap.add_argument("--port", help="아두이노 시리얼 포트 지정")
    args = ap.parse_args(argv)

    if args.list or not args.test:
        print("곡 목록:")
        for name in sorted(tunes.TUNES):
            print(f"  {name:10s} {tunes.duration_ms(name) / 1000:.1f}s")
        if not args.test:
            return

    conf = cfg.SoundConfig(port=args.port) if args.port else cfg.SOUND
    backend = make_backend(conf, prefer_pc=args.pc)
    print("백엔드:", backend.describe)

    try:
        tunes.get(args.test)
    except KeyError as exc:
        raise SystemExit(str(exc)) from exc

    print(f"재생: {args.test} ({tunes.duration_ms(args.test) / 1000:.1f}s)")
    backend.play(args.test)
    time.sleep(tunes.duration_ms(args.test) / 1000 + 0.5)
    backend.close()


if __name__ == "__main__":
    _main(sys.argv[1:])
