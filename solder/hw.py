# -*- coding: utf-8 -*-
"""hw.py — 하드웨어 해석 + mock(print 대체). 납땜 실습 스크립트의 하드웨어 진입점.

왜 있나:
  · 포트가 자주 바뀐다(ttyACM1→ACM0…). 하드코딩하지 않고 여기서 한 번에 해석한다.
  · 팀이 하드웨어 1세트를 공유하며 각자 개인 브랜치에서 테스트한다 → 포트/설정은
    사람마다 다르므로 '커밋 안 하는 개인 설정'(data/local.json·환경변수)으로 받는다.
  · 로봇 또는 카메라 중 하나가 없어도(교체·분리) 코드가 끝까지 돌아야 한다
    → 없는 쪽을 print() 로 흉내내는 Mock 으로 자동 대체한다.

포트 해석 우선순위:
    make_robot(port=...)  >  env SOARM_PORT  >  data/local.json "port"
      >  자동탐지(/dev/ttyACM* · /dev/ttyUSB*)  >  기본 /dev/ttyACM0

Mock 강제:
    env SOARM_MOCK=1        → 로봇·카메라 모두 mock
    env SOARM_MOCK_ROBOT=1  → 로봇만 mock
    env SOARM_MOCK_CAMERA=1 → 카메라만 mock
    data/local.json {"mock_robot": true, ...} 로도 가능
  아무 설정도 없으면 실물 연결을 시도하고, 실패하면 자동으로 mock 으로 떨어진다.

사용:
    from hw import make_robot, make_camera
    be = make_robot()                 # 실물 or MockRobot (be.drv, be.grip 동일 인터페이스)
    with make_camera() as cam: ...     # 실물 or MockCamera (read/read_blocking 동일)
"""
import glob
import json
import os
import random
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

LOCAL_CFG = os.path.join(_ROOT, "data", "local.json")
DEFAULT_PORT = "/dev/ttyACM0"
POS_CENTER = 2048            # STS3215: 0..4095 = 한 바퀴, 중앙 2048 (driver_sdk 와 동일)


# ── 개인 설정 · 포트 해석 ─────────────────────────────────────────────────────
def load_local():
    """data/local.json (gitignore 됨) 개인 설정. 없으면 빈 dict."""
    try:
        with open(LOCAL_CFG) as f:
            return json.load(f)
    except (FileNotFoundError, ValueError):
        return {}


def resolve_port(explicit=None):
    if explicit:
        return explicit
    if os.environ.get("SOARM_PORT"):
        return os.environ["SOARM_PORT"]
    cfg = load_local().get("port")
    if cfg:
        return cfg
    found = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if found:
        return found[0]
    return DEFAULT_PORT


def _forced_mock(kind):
    """env/local.json 으로 mock 이 강제됐는지. kind = 'robot' | 'camera'."""
    if os.environ.get("SOARM_MOCK") == "1":
        return True
    if os.environ.get(f"SOARM_MOCK_{kind.upper()}") == "1":
        return True
    return bool(load_local().get(f"mock_{kind}", False))


# ── Mock 로봇 (print 대체) ───────────────────────────────────────────────────
class MockDriver:
    """STS3215Driver 의 '납땜 스크립트가 쓰는' 메서드만 흉내낸다. 전부 print.
    의존성 없음(serial·mujoco 불필요) — 하드웨어 완전 부재에서도 로직 검증 가능."""

    def __init__(self):
        self.positions = {i: POS_CENTER for i in range(1, 7)}
        self.torque = {i: False for i in range(1, 7)}
        self._steps = 0
        # env SOARM_MOCK_CONTACT=<n> 이면 접촉 자세 하강 n스텝 뒤 부하가 뛰는 걸 흉내낸다
        # (하드웨어 없이 '접촉 감지' 코드경로까지 데모하고 싶을 때). 0=비활성.
        self._contact_at = int(os.environ.get("SOARM_MOCK_CONTACT", "0"))

    # 정적 변환(하드웨어 무관·순수 계산) — driver_sdk 와 동일 식
    @staticmethod
    def position_to_degrees(pos):
        return None if pos is None else round(((pos - POS_CENTER) / 4095.0) * 360.0, 2)

    @staticmethod
    def degrees_to_position(deg):
        return int(POS_CENTER + (deg / 360.0) * 4095)

    def ping(self, sid):
        print(f"[MOCK-ROBOT] ping({sid}) → True")
        return True

    def set_torque(self, sid, enable):
        self.torque[sid] = enable
        print(f"[MOCK-ROBOT] set_torque(id{sid}, {enable})")
        return True

    def set_all_torque(self, enable):
        for i in range(1, 7):
            self.torque[i] = enable
        print(f"[MOCK-ROBOT] set_all_torque({enable})")

    def set_speed(self, sid, v):
        print(f"[MOCK-ROBOT] set_speed(id{sid}, {v})"); return True

    def set_acceleration(self, sid, v):
        print(f"[MOCK-ROBOT] set_acceleration(id{sid}, {v})"); return True

    def set_position(self, sid, pos):
        self.positions[sid] = pos
        print(f"[MOCK-ROBOT] set_position(id{sid}, {pos})"); return True

    def set_all_positions(self, positions):
        for sid, pos in positions.items():
            if pos is not None:
                self.positions[sid] = pos
        self._steps += 1
        deg = {s: round(self.position_to_degrees(p), 1) for s, p in positions.items()}
        print(f"[MOCK-ROBOT] set_all_positions {deg}")

    def get_position(self, sid):
        return self.positions.get(sid)

    def get_all_positions(self):
        return dict(self.positions)

    def get_load(self, sid):
        # 기본은 0 근처 노이즈(오검출 없음). SOARM_MOCK_CONTACT 로 접촉 급증 흉내.
        if self._contact_at and self._steps >= self._contact_at and sid in (2, 3):
            return (self._steps - self._contact_at + 1) * 180
        return random.randint(-15, 15)


class MockRobot:
    """RealBackend 와 같은 인터페이스(be.drv · be.grip · move/ee/wait). 전부 print."""

    def __init__(self):
        self.drv = MockDriver()
        print("[MOCK-ROBOT] print 대체 백엔드 사용 (실물 미연결)")

    def grip(self, frac, speed=400, acc=20):
        print(f"[MOCK-ROBOT] grip({float(frac):.2f})  (0=닫힘 1=열림)")

    def move(self, angles_deg, grip=None, secs=None):
        print(f"[MOCK-ROBOT] move({[round(a,1) for a in angles_deg]}, secs={secs})")

    def ee(self):
        return np.zeros(3)

    def wait(self):
        pass


# ── Mock 카메라 (print 대체) ─────────────────────────────────────────────────
class MockCamera:
    """CameraReader 와 같은 인터페이스. 합성 프레임을 돌려주고 frame_id 를 올린다.
    env SOARM_MOCK_CAM_FREEZE=1 이면 frame_id 를 고정 → 프레임 지연/정지 워치독 데모."""

    def __init__(self, w=640, h=480):
        self.w, self.h = w, h
        self._fid = 0
        self._freeze = os.environ.get("SOARM_MOCK_CAM_FREEZE") == "1"
        print(f"[MOCK-CAMERA] print 대체 카메라 사용 (freeze={self._freeze})")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _frame(self):
        img = np.zeros((self.h, self.w, 3), np.uint8)
        # 눈으로 볼 수 있게 움직이는 표식 하나(검출/서보 스크립트가 뭔가 잡게)
        cx = int(self.w / 2 + 40 * np.sin(self._fid / 10.0))
        try:
            import cv2
            cv2.circle(img, (cx, self.h // 2), 18, (0, 0, 255), -1)
        except Exception:
            img[self.h // 2 - 18:self.h // 2 + 18, cx - 18:cx + 18] = (0, 0, 255)
        depth = np.full((self.h, self.w), 300, np.uint16)
        return img, depth

    def read(self):
        if not self._freeze:
            self._fid += 1
        img, depth = self._frame()
        return img, depth, self._fid

    def read_blocking(self, last_frame_id=0, timeout=1.0, poll_hz=200.0):
        time.sleep(min(0.02, timeout))
        return self.read()

    def close(self):
        pass


# ── 팩토리 ───────────────────────────────────────────────────────────────────
def make_robot(mock=None, port=None):
    """실물 RealBackend 또는 MockRobot. mock=None 이면 env/설정 보고 자동,
    실물 연결 실패 시 자동으로 MockRobot 으로 대체한다."""
    if mock is None:
        mock = _forced_mock("robot")
    if mock:
        return MockRobot()
    port = resolve_port(port)
    try:
        from soarm_lab.real import RealBackend
        be = RealBackend(port=port)
        if not be.drv.ping(1):
            raise RuntimeError("id1 ping 실패")
        print(f"[HW] 실물 로봇 연결됨: {port}")
        return be
    except Exception as e:
        print(f"[HW] 실물 로봇 연결 실패 ({port}: {e}) → MOCK(print) 로 대체")
        return MockRobot()


def make_camera(mock=None):
    """실물 CameraReader 또는 MockCamera. 규칙은 make_robot 과 동일."""
    if mock is None:
        mock = _forced_mock("camera")
    if mock:
        return MockCamera()
    try:
        from hp60c_camera import CameraReader
        cam = CameraReader()            # shm 없으면 예외 → mock 대체
        print("[HW] 실물 카메라 연결됨 (shm bridge)")
        return cam
    except Exception as e:
        print(f"[HW] 실물 카메라 연결 실패 ({e}) → MOCK(print) 로 대체")
        return MockCamera()
