# -*- coding: utf-8 -*-
"""11_replay.py — 10_teach.py 로 만든 순서표를 오픈루프로 재생한다 (파지→접촉).

data/solder_poses.json 의 자세를 순서대로 재생한다. 모든 이동은 안전 골격을 통과한다:
  · 긴급정지  : 언제든 Enter 또는 Ctrl-C → 즉시 토크 해제
  · 관절 한계 : 한계를 넘는 목표 자세는 '거부'하고 멈춘다 (limit 데모, §10.5)
  · delta clamp: 한 스텝당 관절 이동량 제한 → 팔이 홱 튀지 않음
  · 부하 가드  : '접촉' 단계에선 부하 급증 시 목표 전에 정지(=접촉), 과부하면 즉시정지
  · (카메라 켜져 있으면) 프레임 지연/정지 워치독

접촉 임계(safety.LOAD_CONTACT)는 반드시 실측해 맞춘다:
  MODE="probe" 로 접촉 자세를 아주 천천히 내리며 부하를 찍어 보고,
  공회전값과 닿는 순간값 사이로 LOAD_CONTACT 를 잡은 뒤 MODE="replay".

⚠️ 이 스크립트는 팔을 실제로 움직인다. 작업면에서 손을 치우고, Enter(정지)에 손을 얹어둘 것.
"""
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for p in (_ROOT, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from hw import make_robot              # 실물/mock 자동 (포트는 env·local.json 로 해석)
import safety

MODE = "replay"            # "replay"(전체 재생) 또는 "probe"(접촉자세 서행 + 부하로깅)
POSES = os.path.join(_ROOT, "data", "solder_poses.json")
ARM_IDS = (1, 2, 3, 4, 5)

SPEED = 400                # 서보 속도(작을수록 느림 — 접촉 안전 위해 낮게)
ACC = 20
STEP_SLEEP = 0.15          # delta clamp 스텝 사이 대기(초)


# ── 저수준 이동 ──────────────────────────────────────────────────────────────
def read_deg(drv):
    pos = drv.get_all_positions()
    return [drv.position_to_degrees(pos.get(i)) or 0.0 for i in ARM_IDS]


def send(drv, deg5):
    drv.set_all_positions({i + 1: drv.degrees_to_position(deg5[i]) for i in range(5)})


def prep(drv):
    """현재 실측 자세를 목표로 써서 홱 튐 없이 토크를 켠다(§L1 안전기동)."""
    cur = read_deg(drv)
    send(drv, cur)                     # 목표=현재자세
    for i in ARM_IDS:
        drv.set_torque(i, True)
        drv.set_acceleration(i, ACC)
        drv.set_speed(i, SPEED)
    return cur


def glide_to(drv, cur, tgt, est, guard=None, watch_contact=False):
    """cur→tgt 를 delta-clamp 스텝으로 이동. 매 스텝 안전검사.
    watch_contact=True 면 부하 급증 시 목표 전에 멈춘다(접촉). 반환 (cur, contacted)."""
    bad = safety.check_joint_limits(tgt)
    if bad:                             # 한계 초과 목표는 거부(§10.5 데모)
        raise safety.Stopped(f"관절 한계 초과로 목표 거부: {bad}")
    while True:
        est.check()                    # 긴급정지
        if guard is not None:
            if guard.overloaded():
                raise safety.Stopped(f"과부하(부하≥{guard.abort}) — 즉시정지")
            if watch_contact:
                rise = guard.peak_rise()
                print(f"    부하상승 {rise:+d} (접촉임계 {guard.contact})")
                if rise >= guard.contact:
                    return cur, True   # 접촉 → 하강 중단
        reached, nxt = safety.clamp_delta(cur, tgt)
        send(drv, nxt)
        cur = nxt
        time.sleep(STEP_SLEEP)
        if reached:
            return cur, False


# ── 모드 ─────────────────────────────────────────────────────────────────────
def load_poses():
    if not os.path.exists(POSES):
        raise SystemExit(f"{POSES} 없음 — 먼저 10_teach.py 로 자세를 녹화하세요.")
    with open(POSES) as f:
        data = json.load(f)
    poses = data.get("poses", [])
    if not poses:
        raise SystemExit("순서표가 비었습니다 — 10_teach.py 에서 s 로 저장했는지 확인.")
    return poses


def replay(be, est):
    drv = be.drv
    poses = load_poses()
    print(f"재생할 자세 {len(poses)}개:")
    for i, p in enumerate(poses):
        tag = " [접촉]" if p.get("contact") else ""
        print(f"  {i+1:2d}. {p['name']:<16} grip={p['grip']:.2f}{tag}")
    print("\n[안전] 작업면에서 손을 치우세요. 정지는 Enter/Ctrl-C.")
    input("재생하려면 Enter(빈줄) 후 바로 시작합니다 > ")
    est.start()                        # 시작 확인 뒤에 긴급정지 watcher 가동(stdin 경쟁 방지)

    cur = prep(drv)
    guard = safety.LoadGuard(drv)
    print("공회전 부하 baseline 측정...")
    guard.baseline()
    print("  baseline:", guard.base)

    for i, p in enumerate(poses):
        est.check()
        contact = bool(p.get("contact"))
        print(f"[{i+1}/{len(poses)}] {p['name']}"
              f"{' (접촉 감시)' if contact else ''} 로 이동")
        if contact:
            guard.baseline(samples=3)      # 이동 직전 기준 재설정
        cur, touched = glide_to(drv, cur, p["deg5"], est,
                                guard=guard, watch_contact=contact)
        be.grip(p["grip"])                 # 자세 도달 후 집게 상태 반영
        time.sleep(0.4)
        if contact and touched:
            print("  ★ 접촉 감지 — 유지 후 후퇴")
            time.sleep(0.6)
    print("완료.")


def probe(be, est):
    """접촉 임계 실측용: 마지막 접촉 자세로 아주 천천히 내리며 부하를 찍는다(자동정지X).
    닿는 순간 부하가 뛰는 값을 읽어 safety.LOAD_CONTACT 를 잡는다. Ctrl-C/Enter 로 정지."""
    drv = be.drv
    poses = load_poses()
    targets = [p for p in poses if p.get("contact")]
    if not targets:
        raise SystemExit("접촉 단계가 없습니다 — 10_teach 에서 접촉단계 y 로 하나 녹화하세요.")
    tgt = targets[-1]
    print(f"probe: '{tgt['name']}' 로 서행 하강하며 부하 로깅. 닿기 직전 Enter/Ctrl-C.")
    input("시작하려면 Enter > ")
    est.start()                        # 확인 뒤 watcher 가동
    cur = prep(drv)
    guard = safety.LoadGuard(drv)
    guard.baseline()
    print("baseline:", guard.base)
    # delta 를 아주 작게: MAX_STEP 임시 축소
    while True:
        est.check()
        reached, nxt = safety.clamp_delta(cur, tgt["deg5"], max_step=2.0)
        send(drv, nxt)
        cur = nxt
        time.sleep(0.25)
        print(f"  부하 {guard.read()}  상승 {guard.peak_rise():+d}")
        if reached:
            print("자세 도달(접촉 안 났으면 자세를 더 낮춰 다시 티칭). 정지.")
            break


def main():
    be = make_robot()                  # 실물 or mock(print). 포트 자동 해석
    drv = be.drv
    if not drv.ping(1):
        raise SystemExit("서보 응답 없음(id1) — 로봇 전원/케이블/포트 확인.")
    # watcher 는 각 모드가 '시작 확인' 입력을 받은 뒤에 켠다(시작 Enter 오인 방지).
    est = safety.EStop(on_stop=lambda: drv.set_all_torque(False))
    try:
        if MODE == "replay":
            replay(be, est)
        elif MODE == "probe":
            probe(be, est)
        else:
            raise SystemExit(f"알 수 없는 MODE: {MODE!r} (replay 또는 probe)")
    except safety.Stopped as e:
        print(f"\n중단: {e}")
        drv.set_all_torque(False)
    except KeyboardInterrupt:
        est.trip("Ctrl-C")


if __name__ == "__main__":
    main()
