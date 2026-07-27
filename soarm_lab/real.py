# -*- coding: utf-8 -*-
"""real.py — 실물 백엔드(시리얼 직결).

시뮬과 같은 인터페이스(move/ee/wait)라 arm.go(real=True) 로 전환된다.
driver_sdk 로 STS3215 서보에 직접 명령한다.
관절 id: 1 base·2 shoulder·3 elbow·4 wrist·5 roll·6 gripper. 각도=도, 0=중앙.
"""
import numpy as np
from fk_core import FKSo101, ARM_JOINT_NAMES
from driver_sdk import STS3215Driver, JOINT_LIMITS, position_from_fraction


class RealBackend:
    def __init__(self, port="/dev/ttyACM1"):
        lim = FKSo101().limits_deg()               # 관절한계(도) — 명령 전 클램프
        self._lo = [lim[n][0] for n in ARM_JOINT_NAMES]
        self._hi = [lim[n][1] for n in ARM_JOINT_NAMES]
        self.drv = STS3215Driver(port=port)
        self.drv.connect()
        self._grip_lim = JOINT_LIMITS[6]           # id6 percent 매핑(min=닫힘 .. max=열림)
        self._grip_ready = False

    def _clamp_arm(self, angles_deg):
        return [float(np.clip(a, lo, hi))
                for a, lo, hi in zip(angles_deg, self._lo, self._hi)]

    def move(self, angles_deg, grip=None, secs=None):
        arm5 = self._clamp_arm(angles_deg)                     # 안전 클램프
        pos = {i + 1: STS3215Driver.degrees_to_position(arm5[i]) for i in range(5)}
        self.drv.set_all_positions(pos)
        # 그리퍼(id6)는 각도가 아니라 fraction(0=닫힘..1=열림) 이라 grip 인자로 받지 않고
        # 아래 grip() 로 따로 제어한다(arm.go 의 grip=도 의미와 섞이지 않게).

    def grip(self, frac, speed=400, acc=20):
        """그리퍼 여닫기. frac: 0=닫힘 .. 1=열림 (id6 percent 매핑).
        speed/acc 는 여닫는 속도(작을수록 부드럽게 · 공을 튕기지 않게). 처음 호출 때만 설정."""
        if not self._grip_ready:
            self.drv.set_torque(6, True)
            self.drv.set_speed(6, speed)
            self.drv.set_acceleration(6, acc)
            self._grip_ready = True
        frac = float(np.clip(frac, 0.0, 1.0))
        self.drv.set_position(6, position_from_fraction(frac, self._grip_lim))

    def ee(self):
        """현재 관절각을 읽어 FK로 손끝 위치를 추정한다(실물은 site 를 못 읽음)."""
        pos = self.drv.get_all_positions()
        arm5 = [STS3215Driver.position_to_degrees(pos.get(i + 1)) or 0.0
                for i in range(5)]
        p, _ = FKSo101().fk_deg(arm5)
        return p

    def wait(self):
        pass
