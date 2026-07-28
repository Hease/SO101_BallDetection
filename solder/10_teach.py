# -*- coding: utf-8 -*-
"""10_teach.py — 손으로 자세를 잡아 순서표를 만든다 (kinesthetic teach).

팔 서보의 토크를 꺼(set_all_torque False) 손으로 자유롭게 움직일 수 있게 한 뒤,
원하는 자세에서 Enter 로 현재 관절각을 기록한다. 집게는 켜둔 채 o/c 로 여닫으며
그 값도 함께 저장한다. 결과는 data/solder_poses.json 에 '순서표'로 저장되고
11_replay.py 가 그대로 재생한다.

권장 녹화 순서(오픈루프 파지→접촉):
  1) 인두기 위 상공        (집게 열림)   →  Enter
  2) 인두기 파지 높이       (집게 열림)   →  Enter
  3) c 로 집게 닫기(인두기 쥠)            →  Enter
  4) 들어올린 자세                        →  Enter
  5) 타겟(드라이버 마킹) 상공             →  Enter
  6) 팁이 타겟에 살짝 닿는 자세 [접촉]    →  Enter  (접촉단계 y)
  7) 후퇴 자세                            →  Enter

⚠️ 팔 토크가 꺼져 있어 손을 놓으면 중력으로 처진다. 기록 순간엔 팔을 잡고 있을 것.
   모터가 스스로 움직이지 않으므로 이 단계는 안전하다(밀고 들어오지 않음).
"""
import json
import os
import sys

# 프로젝트 루트/패키지 경로 (flat import: fk_core 등)
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for p in (_ROOT, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from hw import make_robot             # 실물/mock 자동 (포트는 env·local.json 로 해석)

OUT = os.path.join(_ROOT, "data", "solder_poses.json")
ARM_IDS = (1, 2, 3, 4, 5)


def read_arm_deg(drv):
    """현재 5개 팔 관절각[도]. 못 읽은 관절은 0.0."""
    pos = drv.get_all_positions()
    return [drv.position_to_degrees(pos.get(i)) or 0.0 for i in ARM_IDS]


def read_loads(drv):
    return {i: drv.get_load(i) for i in ARM_IDS}


def show(drv, grip_frac):
    deg = read_arm_deg(drv)
    loads = read_loads(drv)
    print("  관절각(도): " + " ".join(f"{d:+7.1f}" for d in deg))
    print("  부하     : " + " ".join(f"{loads[i]:+7d}" if loads[i] is not None else "    n/a"
                                     for i in ARM_IDS))
    print(f"  집게     : {grip_frac:.2f} (0=닫힘 1=열림)")


def main():
    be = make_robot()                               # 실물 or mock(print). 포트 자동 해석
    drv = be.drv
    if not drv.ping(1):
        raise SystemExit("서보 응답 없음(id1) — 로봇 전원/케이블/포트 확인.")

    # 팔은 손으로 움직이게 토크 오프, 집게는 제어 위해 살려둔다.
    for i in ARM_IDS:
        drv.set_torque(i, False)
    grip_frac = 1.0
    be.grip(grip_frac)                              # 집게 열고 시작

    poses = []
    print(__doc__)
    print(f"저장 위치: {OUT}\n")
    print("명령: [Enter]=현자세 기록 · o=집게열기 · c=집게닫기 · g=집게값입력 · "
          "d=마지막삭제 · s=저장 · q=종료\n")

    while True:
        show(drv, grip_frac)
        cmd = input(f"[{len(poses)}개 기록됨] 명령 > ").strip()

        if cmd == "":                               # 현재 자세 기록
            deg = read_arm_deg(drv)
            name = input("  이름(빈칸=step{}) > ".format(len(poses) + 1)).strip()
            if not name:
                name = f"step{len(poses) + 1}"
            contact = input("  접촉(하강정지) 단계? y/N > ").strip().lower() == "y"
            poses.append({"name": name, "deg5": deg,
                          "grip": grip_frac, "contact": contact})
            print(f"  ✓ 기록: {name}  grip={grip_frac:.2f}  contact={contact}\n")

        elif cmd == "o":
            grip_frac = 1.0
            be.grip(grip_frac)
            print("  집게 열림\n")
        elif cmd == "c":
            grip_frac = 0.3                         # 인두기 쥐는 값(현장서 조정)
            be.grip(grip_frac)
            print(f"  집게 닫힘 (grip={grip_frac})\n")
        elif cmd == "g":
            try:
                grip_frac = float(input("  집게값 0~1 > ").strip())
                be.grip(grip_frac)
                print(f"  집게 {grip_frac:.2f}\n")
            except ValueError:
                print("  0~1 숫자를 입력하세요\n")

        elif cmd == "d":
            if poses:
                print("  삭제:", poses.pop()["name"], "\n")
            else:
                print("  기록 없음\n")
        elif cmd == "s":
            os.makedirs(os.path.dirname(OUT), exist_ok=True)
            with open(OUT, "w") as f:
                json.dump({"poses": poses}, f, indent=2, ensure_ascii=False)
            print(f"  저장 완료: {OUT}  ({len(poses)}개)\n")
        elif cmd == "q":
            if poses and input("저장 안 하고 종료? 저장하려면 s, 그냥 종료 y > ").strip().lower() != "y":
                continue
            break
        else:
            print("  알 수 없는 명령\n")

    print("종료. (팔 토크는 꺼진 상태 — 필요하면 11_replay 가 다시 켠다.)")


if __name__ == "__main__":
    main()
