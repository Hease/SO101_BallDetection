# -*- coding: utf-8 -*-
"""
mojoco_robotarm_sync.py — MuJoCo 시뮬 팔 ↔ 실물 SO-ARM101 를 'IK 핸들'로 동시 제어.

핵심 아이디어:
  뷰어에 마우스로 끌 수 있는 작은 구(mocap '핸들')를 하나 띄운다.
  매 프레임   핸들 위치 → IK → 관절각 → (1) 시뮬 팔  (2) 실물 팔  로 흘려보낸다.
  핸들을 끌면 시뮬이 따라오고, --real 을 켜면 실물도 같은 각도로 따라온다.

핸들 끌기(뷰어 조작):
  1) 핸들 구를 더블클릭해 선택      (선택되면 강조된다)
  2) Ctrl + 오른쪽드래그  → 평면 이동 (수평 xy)
     Ctrl + 왼쪽드래그    → 높이 이동 (수직 z)
  * mocap 바디라 힘이 아니라 '위치'가 바로 옮겨진다 → 그게 IK 목표점이 된다.

실행:
  python mojoco_robotarm_sync.py                 # 시뮬만 (하드웨어 없이 연습)
  python mojoco_robotarm_sync.py --real          # 시뮬 + 실물 동시 (/dev/ttyACM0)
  python mojoco_robotarm_sync.py --real --port /dev/ttyACM1
"""
import argparse
import time

import numpy as np
import mujoco
import mujoco.viewer

from soarm_lab.sim import SCENE
from soarm_lab.ik_core import IKSo101

# ── IK 핸들을 얹은 씬 만들기 ────────────────────────────────────────────────
# 원본 scene.xml 은 그대로 두고, MjSpec 으로 mocap 구 하나만 얹는다(임시파일 X).
def build_model_with_handle(start_xyz):
    spec = mujoco.MjSpec.from_file(SCENE)
    handle = spec.worldbody.add_body(name="ik_handle", mocap=True, pos=list(start_xyz))
    handle.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.012, 0, 0],
        rgba=[1.0, 0.3, 0.2, 0.6],   # 반투명 빨간 구
        contype=0, conaffinity=0,     # 충돌 없음(팔과 안 부딪힘, 순수 표식)
    )
    return spec.compile()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", action="store_true", help="실물 팔에도 같이 명령")
    ap.add_argument("--port", default="/dev/ttyACM0", help="실물 시리얼 포트")
    ap.add_argument("--real-hz", type=float, default=20.0,
                    help="실물로 명령 보내는 주기(Hz). 너무 높이면 시리얼이 막힌다")
    args = ap.parse_args()

    ik = IKSo101()

    # 시작 손끝 위치를 핸들 초기 위치로 (기본자세에서 IK 로 읽어온다)
    seed = [0, 30, -45, 0, 0]                       # 자연스러운 팔꿈치 자세(도)
    start_xyz, _ = ik.fk.fk_deg(seed)               # 이 각도의 손끝 좌표

    model = build_model_with_handle(start_xyz)
    data = mujoco.MjData(model)
    data.ctrl[:5] = np.radians(seed)                # 시작 자세로 팔 세팅
    mid = model.body("ik_handle").mocapid[0]        # mocap 인덱스

    # 실물 백엔드(선택) — 연결 실패해도 시뮬은 계속
    real = None
    if args.real:
        try:
            from soarm_lab.real import RealBackend
            real = RealBackend(port=args.port)
            # 토크 ON + 부드럽게: 서보는 토크가 꺼지면 안 움직이고, 켜는 순간
            # 목표각으로 튈 수 있어 속도/가속도를 낮춰 첫 이동을 순하게 만든다.
            for sid in range(1, 7):
                real.drv.set_acceleration(sid, 30)
                real.drv.set_speed(sid, 800)
            real.drv.set_all_torque(True)
            print(f"[real] 연결·토크 ON: {args.port}")
        except Exception as exc:
            print(f"[real] 연결 실패 → 시뮬만 진행합니다: {exc}")
            real = None

    last_real = 0.0
    real_dt = 1.0 / args.real_hz
    last_deg = np.array(seed, float)                # 마지막으로 성공한 IK 해(도)

    print("핸들을 더블클릭 → Ctrl+오른쪽드래그(수평) / Ctrl+왼쪽드래그(수직)로 끌어보세요.")
    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            target = data.mocap_pos[mid].copy()     # 핸들(마우스로 끈) 위치 = IK 목표

            # 지금 자세로 warm-start → 부드럽게 수렴, 손끝을 핸들로 보내는 관절각
            deg, err = ik.solve(target, seed_deg=list(last_deg))
            if err <= 0.03:                          # 잔차 3cm 이내면 도달 가능으로 보고 적용
                last_deg = np.asarray(deg, float)
            # 도달 불가(팔 범위 밖)면 마지막 유효 자세를 유지 → 튐 방지

            data.ctrl[:5] = np.radians(last_deg)     # 시뮬 팔에 목표각
            mujoco.mj_step(model, data)              # 물리 한 스텝
            v.sync()

            # 실물로 전송(주기 제한) — 시뮬과 똑같은 각도를 그대로 보낸다
            now = time.time()
            if real is not None and (now - last_real) >= real_dt:
                real.move(last_deg)
                last_real = now

            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
