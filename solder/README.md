# 납땜 모사 (비주얼 서보) — 개발/진행 문서

> 팀 공유용 핸드오프. 합류 팀원은 이 문서 → 실행 순서(§4) → print 드라이버로 먼저
> 돌려보기 순으로 온보딩. (브리프 원문 `PROJECT_BRIEF.md`, 통합 설계 `docs/DEV.md`)

---

## 0. 한 줄 요약

**인두기를 집게에 테이프로 강체 고정** → 탑다운 카메라로 **인두기 팁 마커**를
**타겟(드라이버 끝 마커)** 위로 **비주얼 서보**로 정렬 → 부하 감지될 때까지 하강 = **접촉**.
인두기를 "집는"(파지) 동작은 하지 않는다. 핵심 스크립트: [`solder/30_servo.py`](30_servo.py)

## 1. 왜 이 방향인가 (결정 이력)

| 단계 | 내용 |
|---|---|
| 원래 | 인두기를 파지 → 팁을 타겟에 접촉 (브리프 §1) |
| 문제 | **파지 편차가 최대 난제** — 그립마다 자세가 달라져 팁 오차가 지배(§4.1). 원통이라 슬립까지. |
| 결정① | **인두기를 집게에 강체 고정.** 파지 편차·슬립·원통 파지 문제 통째로 소멸. 남는 건 "팁을 타겟에 대기". |
| 결정② | 팁→타겟 정렬은 **탑다운 비주얼 서보(H 재사용)**. 두 마커를 로봇 평면 xy 로 바꿔 오차만큼 이동. 팁 오프셋 계산 불필요(§4.2). "비전 기반 제어" 요구 충족. |

## 2. 하드웨어 추상화 — sorting 드라이버 계층 (팀 합의)

납땜도 **sorting/ports.py·drivers.py** 의 하드웨어 계약을 쓴다(단일 추상화). 로봇/카메라를
**이름으로 갈아끼운다** — `30_servo.py` 상단 상수:

| 상수 | 값 | 의미 |
|---|---|---|
| `ROBOT_DRIVER` | `"so101"` | 실제 팔(`REAL=True`) 또는 헤드리스 시뮬(`REAL=False`) |
|  | `"print"` | 팔 없이 무엇을 할지 출력만(하드웨어 없이 로직 확인) |
| `CAMERA_DRIVER` | `"hp60c"` | 실제 탑다운 카메라(shm 브리지) |
|  | `"replay"` / `"print"` | 저장 사진 재생 / 합성 화면 |

- 이동은 `RobotController.move_to`(실측 작업영역·IK 잔차·관절 클램프 3중 방어),
  긴급정지는 `robot.estop`(토크 안 끔) + `should_abort`(이동 중에도 정지),
  접촉은 `robot.drv` 의 부하 급증(`solder/safety.LoadGuard`, 실물 so101 에서만).
- 그리퍼는 `soarm_lab/real.py` 의 `grip(frac)` 이 정본(팀 합의). 납땜엔 그리퍼를 안 쓴다.

## 3. 셋업 (처음 한 번)

```bash
./hp60c-camera/scripts/start_bridge.sh          # 카메라 브리지(탑다운 프레임)
export SOARM_PORT=/dev/ttyACM0                   # 포트 다르면 지정(기본 ttyACM0)
```
- **캘리브(H)**: 카메라가 움직였으면 `python vision/03_calib_auto.py` 로 `data/H.npy` 재생성.
- **마커색(Lab)**: `python vision/lab_tuner.py` 로 팁·타겟 두 색을 뽑아 `30_servo.py` 의
  `TIP_LAB`/`TARGET_LAB` 에 입력(클릭하면 그 점 Lab 출력).
- **마커 부착**: 인두기 팁에 색 A, 드라이버 끝에 색 B. 서로/환경색(분홍·노랑·주황)과 안 겹치게.

## 4. 실행 순서

```bash
# 하드웨어 없이 먼저: 30_servo.py 에서 ROBOT_DRIVER=CAMERA_DRIVER="print" 로 두고 확인
# 1) 검출/오차 확인 (MODE="preview", 로봇 안 움직임)
python solder/30_servo.py
# 2) 실제 서보 (MODE="servo") — 정렬(폐루프) → 접촉까지 하강 → 후퇴. 정지: Enter/Ctrl-C
python solder/30_servo.py
```

### 튜닝
- **발산**(팁이 멀어짐) → `GAIN` 부호 뒤집기. 진동하면 `GAIN`↓.
- 접촉 임계 `safety.LOAD_CONTACT` : 하강 로그의 "부하상승" 을 보고 공회전값과 닿는 값 사이로.
- `STEP_CLAMP`(1스텝 최대 이동)·`Z_STEP`(하강 간격)·`Z_MIN`(하강 바닥).

## 5. 파일 구조

| 파일 | 역할 |
|---|---|
| [`solder/30_servo.py`](30_servo.py) | **메인** — 탑다운 비주얼 서보 + 접촉 |
| `solder/safety.py` | 서보 전용 안전 헬퍼(키보드 E-STOP·부하 접촉 가드·프레임 워치독) |
| `vision/lab_tuner.py` | Lab 색 튜너(마커색 잡기) |
| `vision/03_calib_auto.py` | H 캘리브(픽셀→로봇평면) |
| `sorting/` (드라이버·robot·safety·workspace·watchdog·calib·tests) | 공용 인프라 — 여기에 얹혀 돈다 |

> **통합 정리(2026-07-28)**: 하드웨어 추상화는 `sorting` 드라이버로 일원화하며
> `solder/hw.py` 제거. 납땜 정본은 `30_servo` 로 정하며 `sorting/solder.py`(상태머신)와
> 구 파지 스크립트(`10_teach`·`11_replay`·`20_locate`·`21_pick`) 제거.

## 6. 안전 / 과제 요구사항

| 요구사항(브리프 §8) | 구현 |
|---|---|
| 긴급정지 | `safety.EStop`(Enter/Ctrl-C) → `robot.estop`(토크 유지) + `should_abort` |
| 카메라 지연/끊김 | `safety.FrameWatchdog` — frame_id 정지 시 중단 |
| 로봇 limit | `RobotController.move_to` 3중 방어(작업영역·IK 잔차·관절 클램프) |
| 접촉/과부하 | `safety.LoadGuard` — 부하 급증=접촉, 절대한계=즉시정지 |
| 비전 기반 제어 | `30_servo` 폐루프 자체 |

## 7. 현재 상태 / 다음

- ✅ 서보 파이프라인이 `sorting` 드라이버 위에서 동작(print 드라이버로 스모크 검증).
  전체 회귀 `pytest` green(134 passed).
- ⬜ **실물 검증**: 마커 두 색 Lab 확정 / 서보 `GAIN` 부호·게인 / 접촉 `LOAD_CONTACT` 실측.
- ⬜ preview 로 두 마커·오차 방향 먼저 확인 → servo.

## 8. 팀 협업

- 개인 설정(`data/local.json`·`H.npy`·`*.json`)은 `.gitignore` 처리 — `git switch` 해도 안 섞임.
- 각자 H 는 로컬에만(`vision/03_calib_auto.py` 로 생성).
- 온보딩: `git fetch && git switch <브랜치>` → `30_servo.py` 를 print 드라이버로 먼저 돌려보기.
