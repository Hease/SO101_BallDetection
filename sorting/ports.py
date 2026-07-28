# -*- coding: utf-8 -*-
"""ports.py — 하드웨어가 갖춰야 할 **계약**. 바뀌는 것과 안 바뀌는 것을 가른다.

카메라나 로봇이 바뀌어도 `vision` · `pipeline` · `workspace` · `watchdog` 은
한 줄도 고치지 않는다. 그러려면 "위쪽 코드가 하드웨어에게 무엇을 요구하는가"가
한곳에 적혀 있어야 한다. 그게 이 파일이다.

    ┌─ 안 바뀌는 쪽 ──────────────────────────────┐
    │  pipeline · vision · workspace · watchdog   │
    └───────────────┬─────────────────────────────┘
                    │  RobotPort · CameraPort  (이 파일)
    ┌───────────────┴─────────────────────────────┐
    │  so101 / print / …    hp60c / replay / print │   ← 갈아끼우는 쪽
    └─────────────────────────────────────────────┘

Protocol 을 쓰는 이유: 새 드라이버가 이 클래스를 **상속할 필요가 없다**.
메서드 이름과 모양만 맞으면 된다. 남의 라이브러리를 감싸는 어댑터를 만들 때
상속을 강요하면 오히려 걸리적거린다.

`@runtime_checkable` 이라 `isinstance(obj, RobotPort)` 로 검사할 수 있고,
`tests/test_ports.py` 가 등록된 모든 드라이버에 대해 그 검사를 돌린다.
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class RobotPort(Protocol):
    """파이프라인이 로봇에게 요구하는 전부.

    새 로봇을 붙이려면 이 메서드들만 구현하면 된다. IK 를 어떻게 푸는지,
    시리얼인지 이더넷인지, 관절이 몇 개인지는 위쪽 코드의 관심사가 아니다.
    """

    # 이동 중 "지금 멈춰야 하나"를 물어보는 훅. 파이프라인이 여기에 침입 감지와
    # 딜레이 워치독을 연결한다. 구현체는 대기 루프에서 주기적으로 불러야 한다.
    should_abort: object

    @property
    def estopped(self) -> bool:
        """지금 비상정지 상태인가."""

    def home(self) -> None:
        """안전 대기 자세로."""

    def move_to(self, xyz, down: bool = False, wait: bool = True) -> None:
        """손끝을 좌표(m)로. 도달할 수 없으면 OutOfReach 를 올린다."""

    def pick(self, target_xy, attempt: int = 0) -> None:
        """그 자리의 물체를 집는다. attempt 는 재시도 횟수(하강 높이 보정용)."""

    def place(self, target_xy) -> None:
        """지금 들고 있는 것을 그 자리에 놓는다."""

    def set_grip(self, pct: float, settle: float = 0.0) -> None:
        """그리퍼를 pct(0=닫힘, 100=열림)로."""

    def holding_object(self) -> bool:
        """지금 무언가를 물고 있는가. 파지 성공 판정에 쓴다."""

    def jog(self, joint_index: int, delta_deg: float) -> None:
        """관절 하나를 상대 이동. 수동 모드 전용."""

    def joint_angles_deg(self) -> list[float]:
        """3D 트윈에 보낼 관절각. 비싸면 안 된다(자주 불린다)."""

    def estop(self) -> None:
        """즉시 정지. **토크를 끄지 말 것** — 팔이 주저앉으면 더 위험하다."""

    def release_estop(self) -> None:
        """정지 해제."""

    def close(self) -> None:
        """자원 정리."""


@runtime_checkable
class CameraPort(Protocol):
    """파이프라인이 카메라에게 요구하는 전부.

    `hp60c_camera.CameraReader` 의 모양을 그대로 따랐다 — 이미 잘 동작하는
    인터페이스를 새로 만들 이유가 없고, 기존 리더가 그대로 어댑터가 된다.
    """

    def read_blocking(self, last_frame_id: int = 0, timeout: float = 1.0,
                      poll_hz: float = 200.0):
        """새 프레임을 기다려 (rgb, depth, frame_id) 로 돌려준다.

        rgb   : BGR uint8 (H, W, 3)
        depth : uint16 mm (H, W) 또는 None — **깊이가 없어도 된다.**
                안전감시가 유휴가 될 뿐 나머지는 그대로 돈다.
        """

    def __enter__(self): ...
    def __exit__(self, *exc): ...


def describe(obj) -> str:
    """드라이버가 자기를 뭐라고 소개하는지. GUI 와 로그에 그대로 쓴다."""
    return getattr(obj, "describe", None) or type(obj).__name__


def missing_methods(obj, port) -> list[str]:
    """계약에서 빠진 메서드 목록. 새 드라이버를 붙일 때 뭘 빠뜨렸는지 알려준다.

    `isinstance` 는 되고 안 되고만 말해주지, **무엇이 없는지**는 말해주지 않는다.
    새 하드웨어를 붙이는 사람에게는 그게 알고 싶은 전부다.
    """
    required = [n for n in dir(port)
                if not n.startswith("_") and n not in ("should_abort",)]
    return [n for n in required if not hasattr(obj, n)]


def blank_frame(width: int = 640, height: int = 480) -> np.ndarray:
    """드라이버가 아직 프레임을 못 줄 때 쓸 검은 화면."""
    return np.zeros((height, width, 3), dtype=np.uint8)
