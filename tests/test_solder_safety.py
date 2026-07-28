# -*- coding: utf-8 -*-
"""납땜 서보의 긴급정지 검증.

여기서 지키려는 것: **정지는 사람이 눌렀을 때만 걸린다.**

E-STOP 이 안 걸리는 것도 위험하지만, 걸려야 할 이유가 없는데 걸리는 것도
그만큼 나쁘다. 시연 중에 원인 모를 정지가 나면 그게 진짜 위험 감지인지
버그인지 구분할 수 없고, 그러면 사람이 E-STOP 을 믿지 않게 된다.
"""
from __future__ import annotations

import os
import sys
import time

import pytest

_SOLDER = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "solder")
if _SOLDER not in sys.path:
    sys.path.insert(0, _SOLDER)

import safety  # noqa: E402  (경로를 넣은 뒤에야 import 된다)


@pytest.fixture
def fake_stdin():
    """진짜 fd 를 가진 stdin 대역. select() 를 쓰므로 StringIO 로는 안 된다."""
    read_fd, write_fd = os.pipe()
    original = sys.stdin
    sys.stdin = os.fdopen(read_fd, "r")
    writer = os.fdopen(write_fd, "w")
    try:
        yield writer
    finally:
        sys.stdin.close()
        sys.stdin = original
        if not writer.closed:
            writer.close()


def _settle(est, timeout=1.0):
    """watcher 스레드가 stdin 을 한 번 읽을 시간을 준다."""
    deadline = time.time() + timeout
    while time.time() < deadline and not est.stopped:
        time.sleep(0.02)


def test_enter_trips_the_estop(fake_stdin):
    est = safety.EStop().start()
    fake_stdin.write("\n")
    fake_stdin.flush()

    _settle(est)
    assert est.stopped, "Enter 를 눌렀는데 정지가 안 걸렸다"
    with pytest.raises(safety.Stopped):
        est.check()


def test_eof_does_not_trip_the_estop(fake_stdin, capsys):
    """EOF 는 사람이 Enter 를 누른 게 아니다.

    파이프·리다이렉션으로 돌리면(`--yes` 로 무인 실행할 때가 그렇다) select 가
    즉시 readable 을 주고 readline 이 "" 를 돌려준다. 이걸 입력으로 세면
    **시작하자마자 정지**해서 서보가 한 번도 안 돈다.
    """
    est = safety.EStop().start()
    fake_stdin.close()                      # 쓰기 끝 닫음 = EOF

    _settle(est, timeout=0.5)
    assert not est.stopped, "EOF 를 Enter 로 오인해서 정지가 걸렸다"
    est.check()                             # 예외가 나면 안 된다

    # watcher 가 EOF 를 실제로 보고 물러난 것인지 확인한다. select 가 예외로
    # 빠져나가도 위 단언은 통과하므로, 그것만으론 이 테스트가 헛돈다.
    assert "EOF" in capsys.readouterr().out, "EOF 경로를 타지 않았다"


def test_on_stop_runs_once():
    """중복 정지에도 콜백은 한 번만 — estop 을 여러 번 치면 로그가 지저분해진다."""
    calls = []
    est = safety.EStop(on_stop=lambda: calls.append(1))
    est.trip("첫 번째")
    est.trip("두 번째")
    assert calls == [1]


def test_on_stop_failure_still_leaves_it_stopped():
    """콜백이 터져도 정지 상태는 유지된다 — 정지가 콜백에 의존하면 안 된다."""
    def boom():
        raise RuntimeError("시리얼 끊김")

    est = safety.EStop(on_stop=boom)
    est.trip("테스트")
    assert est.stopped
