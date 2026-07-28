# -*- coding: utf-8 -*-
"""모든 테스트를 실제 캘리브레이션에서 떼어 놓는다.

왜 필요한지는 실제로 겪어서 안다. 작업영역 실측 기능을 붙인 뒤 전체 회귀가
세 개 깨졌는데, 원인은 코드가 아니라 **그 컴퓨터에 저장돼 있던 측정값**이었다.
GUI 스모크 테스트가 `data/calibration.json` 에 작업영역을 써 놓았고, 진짜
RobotController 를 쓰는 테스트들이 그 영역 밖 좌표로 팔을 보내다 거절당한 것이다.

이걸 그냥 두면 이렇게 된다:

  · 작업영역을 측정한 팀원의 PC 에서만 테스트가 깨진다
  · 테스트가 실제 캘리브레이션 파일을 덮어써서, 힘들게 잰 값이 날아간다

둘 다 3명이 각자 작업하는 상황에서 시간을 크게 잡아먹는 종류의 문제다.
그래서 **모든 테스트가 임시 저장소를 쓰게** 자동으로 갈아끼운다. 실제 측정값을
쓰고 싶은 테스트는 명시적으로 파일 경로를 넘기면 된다.
"""
from __future__ import annotations

import pytest

from sorting import calib


@pytest.fixture(autouse=True)
def isolate_calibration(tmp_path, monkeypatch):
    """테스트마다 빈 캘리브레이션 저장소를 준다.

    autouse 라 모든 테스트에 자동 적용된다. 테스트가 저장소에 무엇을 쓰든
    tmp_path 안에서 끝나므로 실제 `data/calibration.json` 은 손대지 않는다.
    """
    store = calib.CalibStore(path=str(tmp_path / "calibration.json"))
    monkeypatch.setattr(calib, "STORE", store)
    return store
