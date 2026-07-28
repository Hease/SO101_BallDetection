# 개발 문서 (내부 공유용)

## 이 시스템이 무엇을 하는가

**주 과제 — 납땜 모사.** 인두기(가열하지 않은 더미)를 집게에 강체 고정해 두고,
탑다운 카메라가 **팁 마커**와 **타겟 마커**를 함께 본다. 두 점을 로봇 평면 xy 로
바꿔 **오차만큼 팔을 움직이는 폐루프**로 정렬한 뒤, 부하가 뛸 때까지 하강 = 접촉.
정본은 [`solder/30_servo.py`](../solder/30_servo.py), 진행 문서는
[`solder/README.md`](../solder/README.md).

**부수 과제 — 색 분류.** `sorting/pipeline.py` 의 공 분류 파이프라인. 지금은 주
시나리오가 아니지만 **회귀 기준선**으로 남겨 뒀다: 상태머신·트래킹·4단계 워치독·
GUI 가 여기에만 있고, 테스트 139개 중 상당수가 이걸 통해 안전 배선을 검증한다.
지우면 그 검증도 같이 사라진다.

## 계층 구조

두 과제가 **같은 하부 구조 위에** 얹혀 있다. 위(과제)만 다르고 아래는 공유한다.

```
    납땜 (주)                        색 분류 (회귀 기준선)
    solder/30_servo.py               sorting/pipeline.py
    ├ 비주얼 서보 폐루프              ├ 상태머신
    └ solder/safety.py               └ sorting/tracking.py
      (EStop·LoadGuard·FrameWatchdog)   sorting/gui/ · watchdog.py
 ───────────────┬────────────────────────────┬─────────────────────
                ↓        공유 하부 구조        ↓
    sorting/ports.py · drivers.py   ← 하드웨어 계약 + 어댑터
    sorting/robot.py                ← 좌표 이동 · 3중 방어 · E-STOP
    sorting/workspace.py            ← 실측 작업영역
    sorting/calib.py                ← 실측값 저장소(출처 추적)
    sorting/vision.py · mapping.py  ← Lab 검출 · 4점 호모그래피
    soarm_lab/                      ← IK · 시뮬/실물 백엔드
```

Intel RVC 참고 구조와도 대응된다(발표에서 이 대응을 쓰면 설명이 쉽다).

```
   RVC                          우리 코드
 ─────────────────────────────────────────────────────────
   Sensing                      hp60c-camera (RGB-D)
   Vision                       vision.py · 30_servo.detect_both
   (좌표 변환)                   mapping.py · H (4점 호모그래피)
   Motion Control Interface     robot.py     ← RVC 의 motion_controller_interface 자리
   Use Case / Task              30_servo.servo · pipeline.py
   Manipulation                 soarm_lab (IK · 시뮬/실물 백엔드)
 ─────────────────────────────────────────────────────────
   안전은 계층을 가로지른다:  solder/safety.py · workspace.py · watchdog.py
```

## 값이 어디서 오는가 — 이 프로젝트의 규칙

> **잴 수 있으면 재고, 잰 값은 파일에 남기고, 화면에 출처를 같이 보여준다.**

이 규칙이 생긴 이유가 있다. 초기에 주황 구역 색은 촬영샷에 없어서 눈대중으로
넣었고, 도달 한계는 아예 없어서 IK 잔차에 맡겼다가 **반경 0.44m 까지 통과**하는
걸 뒤늦게 알았다. 둘 다 "실측 가능했는데 안 잰" 경우였다.

| 어디에 | 무엇이 |
|---|---|
| `data/calibration.json` | 실측·수동 입력값 (git 에 안 올린다 — 설치마다 다름) |
| `sorting/config.py` | 기본값과 순수 튜닝 노브(속도·커널 크기·프레임 카운트) |

```bash
python -m sorting.calib report     # 무엇이 측정/수동/기본인지 + 재는 방법
```

출처는 셋 중 하나다. `measured`(자동 측정) → `manual`(사람이 넣음) → `default`
(아직 아무도 안 잼). **`manual` 이 있는 이유는 하드웨어 때문이다.** 서보가
응답하지 않아도 데모는 해야 하므로, 자로 잰 값을 넣고 진행하되 그 사실과 사유가
파일에 남는다.

### 이 숫자는 어떻게 재나

| 값 | 재는 명령 | 방법 |
|---|---|---|
| 작업영역 경계 | `teach limits` | 토크 끄고 안전 범위 테두리를 손으로 한 바퀴 |
| z_grasp / z_hover / z_release | `teach heights` | 팔끝을 작업면에 대고 Enter |
| 홈 자세 | `teach home` | 안전한 대기 자세를 손으로 잡고 Enter |
| 좌표 등록 | `teach add <이름>` | 팔끝을 그 지점에 대고 Enter |
| 그리퍼 열림/닫힘/빈손기준 | `teach grip` | 실제로 물려보고 읽음 (분류 전용) |
| Lab 기준색 | `lab_sample` · `vision/lab_tuner.py` | 대상을 클릭 → 붙여넣을 한 줄이 나옴 |
| 픽셀→로봇 변환 H | `vision/03_calib_auto.py` | 카메라가 움직였으면 필수 |
| 안전 경계선·테이블 기준면 | `setup_zone` | 작업영역 클릭 + 빈 테이블 촬영 |
| 아무거나 (수동) | `calib set <키> <값> --note "사유"` | 자동이 막혔을 때 |

납땜 쪽은 아직 `calib` 를 안 거치고 **`30_servo.py`·`solder/safety.py` 상단
상수**에 있다. 옮기기 전까지는 그 파일에서 직접 고친다.

| 값 | 어디에 | 어떻게 정하나 |
|---|---|---|
| `TIP_LAB` / `TARGET_LAB` | `30_servo.py` | `vision/lab_tuner.py` 로 마커를 클릭 |
| `GAIN` | `30_servo.py` | preview 로 오차 방향 확인 → 발산하면 부호 반전, 진동하면 ↓ |
| `XY_TOL` (정렬 완료) | `30_servo.py` | 요구 정밀도. 기본 4mm |
| `STEP_CLAMP` · `Z_STEP` · `Z_MIN` | `30_servo.py` | 안전 상한. 한 번에 크게 움직이지 않게 |
| `LOAD_CONTACT` | `solder/safety.py` | **실측 필수** — 하강 로그의 "부하상승"에서 공회전값과 닿는 값 사이 |
| `LOAD_ABORT` | `solder/safety.py` | 과부하 절대한계 |
| `FRAME_STALL_MS` | `solder/safety.py` | 프레임 정지 판정 (기본 300ms) |

> ⬜ **미완**: 위 표의 값들은 실측 대상인데 아직 `calib.MEASURABLE` 에 없다.
> `LOAD_CONTACT` 는 인두기 무게가 바뀌면 다시 잡아야 하므로 특히 그렇다.

## 안전 3중 방어 — 각 층이 잡는 것이 다르다

`robot.move_to` 가 순서대로 통과시킨다.

1. **실측 작업영역** (`workspace.py`) — IK 를 부르기 *전에* 거절. 빠르고 예측
   가능하며 **화면에 그릴 수 있다**. 이 층이 없으면 사실상 한계가 없다:
   IK 는 반경 0.44m 까지 잔차 0mm 로 풀고 관절한계도 안 넘긴다.
2. **IK 잔차** (`OutOfReach`) — 영역 안이지만 그 자세로는 못 푸는 경우.
3. **관절 클램프** (`RealBackend._clamp_arm`) — 벤더 코드에 이미 있는 최후 방어.

측정 전에는 `Workspace.load()` 가 `None` 을 준다. **그럴듯한 기본 영역을 지어
내지 않는다** — 없는 한계를 있는 척하면 "검사가 돌고 있다"고 착각하게 된다.
대신 GUI 가 "작업영역 미측정"을 빨갛게 띄운다.

## 딜레이 보호장치 (`watchdog.py`)

지금까지 지연을 **재기만** 했다. 재는 것과 대응하는 것은 다르다 — 1초 늦은
프레임의 좌표로 팔을 보내면 사람이 이미 손을 뻗은 자리로 간다.

```
LEVEL 0 정상  < 300ms
LEVEL 1 경고  300~800ms    화면 경고 + 감속
LEVEL 2 보류  800~1500ms   신규 동작 금지 (진행 중인 동작은 마무리)
LEVEL 3 정지  > 1500ms     estop()
```

- **악화는 즉시, 회복은 한 단계씩.** 목표 단계로 곧장 뛰면 900→400→200ms 처럼
  나쁜 값이 섞인 구간에서 '개선 3회'로 세어져 정지에서 정상으로 건너뛴다.
- LEVEL 3 은 `robot.should_abort` 통로를 탄다 — 도착 대기 루프가 20ms 마다
  검사하므로 **동작 한복판에서도** 선다. 침입 감지와 같은 경로다.
- 임계값도 실측 대상이다. `measure_thresholds()` 로 정상 지연 분포의 p95 에서
  뽑을 수 있다.

### ⚠️ 서보 루프는 아직 이진 판정이다

**`sorting/watchdog.py` 는 분류 파이프라인에만 물려 있다.** 납땜 서보 루프
(`30_servo.servo`)는 `solder/safety.FrameWatchdog` 를 쓰는데, 이건 4단계가 아니라
**"frame_id 가 300ms 넘게 안 바뀌면 중단"** 하나뿐이다.

```python
if rgb is None or wd.stalled(last):
    raise safety.Stopped("카메라 프레임 지연/정지")
```

안전 측면에서 틀리지는 않았다 — 늦은 프레임으로 팔을 움직이지 않는다는 목적은
달성한다. 다만 **감속·보류 같은 중간 단계가 없어서** 살짝 느려지면 그냥 선다.

붙이려면: `servo()` 안에서 프레임 나이를 `LatencyWatchdog.report()` 에 넘기고
`may_start_motion()` 이 False 면 `goto` 를 건너뛰면 된다. `should_abort` 통로는
이미 `est.stopped` 로 연결돼 있으므로 거기에 `wd.should_abort()` 를 OR 로 얹는다.

## 납땜 서보의 안전 (`solder/safety.py`)

| 클래스 | 무엇을 막나 |
|---|---|
| `EStop` | stdin 감시 스레드. Enter/Ctrl-C → `robot.estop()` + `should_abort` |
| `LoadGuard` | 관절 2·3(수직 반력) 부하. 기준선 대비 상승 = **접촉**, 절대한계 = **과부하 정지** |
| `FrameWatchdog` | `frame_id` 정지 감시 |
| `clamp_delta` | 한 명령당 관절 최대 이동(기본 8°) — 튀는 명령을 나눠 보낸다 |

접촉을 **위치가 아니라 부하로** 판정하는 것이 핵심이다. "z 가 몇 mm 면 닿는다"는
작업물 두께가 바뀌면 틀리지만, 부하가 뛰는 건 실제로 닿았을 때만 일어난다.

수렴 실패도 안전 문제로 다룬다: `MAX_ITERS`(40회) 안에 못 맞추거나 마커를
`MISS_MAX`(15회) 연속으로 못 찾으면 중단한다. **무한히 시도하지 않는다.**

## 스레드

Qt 메인 스레드는 **그리기만** 한다. 카메라도 로봇도 블로킹 I/O 라 하나라도
메인에 두면 창이 얼고, 그러면 E-STOP 버튼조차 안 눌린다.

```
[CameraWorker]  읽기→검출→트래킹→안전감시→워치독 보고  ──▶ [메인] 그리기
[PipelineWorker] 상태머신 + 로봇 시리얼(단독 소유)      ──▶ [메인] 3D 트윈
```

로봇 시리얼은 PipelineWorker 만 만진다 — `driver_sdk` 는 sync write 없이 관절마다
개별 write 를 하므로 두 스레드가 동시에 쓰면 패킷이 섞인다.
**E-STOP 만 예외**로 메인 스레드가 직접 친다. 큐에 넣고 워커 차례를 기다리면
그 워커가 마침 이동 대기 중일 때 정지가 늦는다.

## 테스트

```bash
pytest -m "not slow"    # 9초 · 129개 — 개발 중에는 항상 이것
pytest                  # 100초 · 139개 — 커밋 전 한 번
```

**빠른 레인이 있는 이유**: 요구사항이 발표 3시간 전에 바뀐다. 10초 피드백과
100초 피드백은 완전히 다른 작업이 된다. `tests/fakes.py` 의 `FakeRobot` 은
물리 없이 즉시 이동하므로 파이프라인 판단을 밀리초에 검증한다.

> ROS 환경이 켜져 있으면 `/opt/ros/*/site-packages` 의 pytest 플러그인이 충돌해
> 수집 단계에서 죽는다. `env -u PYTHONPATH -u AMENT_PREFIX_PATH pytest` 로 돈다.

| 파일 | 지키는 것 |
|---|---|
| `test_vision.py` | 촬영샷 8장의 공 개수, 과자봉지 오검출 |
| `test_pipeline_fast.py` | 순서·분기·재시도 (물리 없이, 빠름) |
| `test_pipeline.py` | 실제 IK·물리까지 (느림, `slow` 마커) |
| `test_safety_integration.py` | **배선** — 모듈이 실제로 물려 있는가 |
| `test_workspace.py` | 안팎 판정, IK 가 통과시키는 0.44m 거절 |
| `test_watchdog.py` | 4단계 전이·히스테리시스·회복 |
| `test_calib.py` | 출처 추적, 수동 덮어쓰기, 파일 깨져도 안 죽음 |
| `test_teach.py` | 훑기→영역, 서보 무응답 시 수동 경로 |
| `test_ports.py` | 등록된 **모든** 드라이버가 계약을 지키는가 (하드웨어 교체) |
| `test_safety.py` | 깊이 기반 침입 감지 |
| `test_gui.py` | 창이 뜨고 워커가 물리는가 (스모크) |

> ⬜ **미완**: `30_servo` 에 대한 테스트가 없다. 순수 함수(`clamp_vec` ·
> `detect_pixel` · `to_robot`)와 수렴 로직은 하드웨어 없이 검증할 수 있다 —
> 가짜 카메라가 "팁이 오차만큼 따라오는" 프레임을 내주면 서보가 수렴하는지까지
> 테스트 가능하다. 지금은 print 드라이버 스모크로만 확인하고 있다.

---

## ⚡ 새 요구사항이 오면 어디를 고치나

발표 3시간 전 대응용. **요구 유형 → 고칠 파일 → 확인 명령** 순서로 본다.

| 요구 유형 | 고칠 곳 | 확인 |
|---|---|---|
| **납땜 지점 여러 개** | `30_servo.servo` 를 타겟 목록으로 감싸는 루프 + 완료한 타겟 마커 색 변경/제거 | print 드라이버로 스모크 |
| **정밀도 요구 강화** | `30_servo.XY_TOL` ↓ (수렴 반복이 늘어나므로 `MAX_ITERS` 도 같이) | preview → servo 로그의 err |
| **접촉 조건 변경** (더 세게/약하게) | `solder/safety.LOAD_CONTACT` | 하강 로그 "부하상승" |
| **마커 색 변경** | `vision/lab_tuner.py` 로 재측정 → `TIP_LAB`/`TARGET_LAB` | preview |
| **접촉 후 동작 추가** (유지시간·경로 등) | `30_servo.servo` 의 하강 루프 뒤 | print 드라이버 |
| **새 안전 규칙** | 공간이면 `workspace.py`, 시간이면 `watchdog.py`/`solder/safety.py` | `test_safety_integration.py` |
| **새 좌표를 손으로 지정** | `teach add <이름>` → `PoseLibrary` 에서 읽어 씀 | `teach list` |
| **속도·정밀도 요구** | `config.RobotConfig` 의 speed/accel/settle_tol | 실물 확인 |
| **하드웨어 교체** | `drivers.py` 에 어댑터 + 등록 한 줄 | `python -m sorting.drivers` |
| **검출 대상 추가** | `vision.detect_region` 재사용 + `Scene` 에 필드 | `test_vision.py` |
| **색 추가** (분류 쪽) | `config.py` 의 `BALL_COLORS`/`BIN_COLORS`/`PLACE_MAP` + `lab_sample` | `pytest -m "not slow"` |
| **분류 규칙 변경** | `pipeline._choose_ball` / `_run_cycle` 의 `bin_color` 결정 | `test_pipeline_fast.py` |
| **UI 표시 항목 추가** | `gui/panel.py` 의 `_build_status` | GUI 실행 |

**작업 순서 권장**: ① `pytest -m "not slow"` 로 현재 초록 확인 → ② 테스트 먼저
한 줄 추가 → ③ 구현 → ④ 빠른 레인 → ⑤ 전체 레인 → ⑥ 커밋.

**변경 영향범위 판정**은 C 트랙(문서·테스트 담당)이 맡는다. 위 표에서 해당
행을 찾고, 그 행의 확인 명령이 초록이면 나머지는 건드리지 않은 것이다.

## 하드웨어가 바뀌면

카메라나 로봇이 바뀌어도 `30_servo` · `vision` · `pipeline` · `workspace` ·
`watchdog` 은 **한 줄도 고치지 않는다.** 위쪽 코드가 하드웨어에게 요구하는 것이
`sorting/ports.py` 에 계약으로 적혀 있고, 실제 장비는 `sorting/drivers.py` 의
어댑터로만 붙는다. **납땜과 분류가 같은 계약을 쓴다**(팀 합의 — `solder/hw.py` 는
이 일원화 과정에서 제거됨).

```bash
# 분류 파이프라인 — CLI 플래그
python sorting_main.py --robot print --camera print
python -m sorting.drivers                       # 등록된 드라이버 계약 점검

# 납땜 서보 — 30_servo.py 상단 상수를 고친다
#   ROBOT_DRIVER  = "print"
#   CAMERA_DRIVER = "print"
python solder/30_servo.py
```

> ⬜ **미완**: 납땜 쪽은 드라이버 선택이 모듈 상수라 **소스를 편집해야** 바뀐다.
> 시연 중 파일 편집은 사고가 나기 쉬우므로 환경변수나 CLI 플래그로 빼는 게 낫다.

| 종류 | 이름 | 무엇 |
|---|---|---|
| 로봇 | `so101` | 실제 팔(시리얼) 또는 헤드리스 MuJoCo |
| 로봇 | `print` | 안 움직이고 무엇을 할지 출력만 |
| 카메라 | `hp60c` | 실제 뎁스카메라 |
| 카메라 | `replay` | 저장된 사진 재생 |
| 카메라 | `print` | 검출 가능한 장면을 합성 |

### 새 하드웨어를 붙이는 법

1. `drivers.py` 에 어댑터 클래스를 쓴다. **`RobotPort` 를 상속할 필요가 없다** —
   메서드 이름과 모양만 맞으면 된다(Protocol).
2. `ROBOTS` 또는 `CAMERAS` 딕셔너리에 한 줄 등록한다.
3. `python -m sorting.drivers` 로 계약을 점검한다. 빠진 메서드를 이름으로 알려준다.
4. `pytest tests/test_ports.py` — 등록된 모든 드라이버에 대해 자동으로 돈다.

### print 드라이버가 그냥 장난감이 아닌 이유

`PrintRobot` 은 **거짓말을 하지 않는다.** 자기 자세를 기억하고, 그리퍼 개도로
파지 성공을 판정하고, E-STOP 을 걸면 실제로 이후 명령을 무시하고, 측정된
작업영역 밖이면 `OutOfReach` 를 올린다. 그래서 위쪽 코드가 진짜 로봇에서와
**같은 경로**를 탄다.

이게 실제로 값어치를 했다. print 로봇으로 갈아끼우자 공 2개가 **42,485개로**
세어졌다 — 놓자마자 재스캔하면 트래커가 아직 그 공을 들고 있어 같은 공을 다시
집는 결함이었다. 지금까지는 "이동에 몇 초 걸리니 그 사이 트래커가 잊는다"에
가려져 있었는데, 그건 *로봇이 트래커보다 느리다*는 가정이다. 하드웨어를 바꾸는
순간 깨진다. `handled_cooldown_s` 로 고쳤고 회귀 테스트로 묶어 두었다.

## 자주 겪는 문제

| 증상 | 볼 곳 |
|---|---|
| **팁이 타겟에서 멀어짐(발산)** | `GAIN` 부호 반전 — H 의 축 방향과 반대인 경우 |
| **팁이 타겟 주위에서 진동** | `GAIN` 을 낮춘다 (0.6 → 0.3) |
| **"마커를 계속 못 찾음"** | `lab_tuner.py` 로 재측정. 조명·그림자, `MIN_AREA` |
| **"40회 안에 수렴 실패"** | 대개 `GAIN` 부호. preview 에서 화살표 방향부터 확인 |
| **접촉을 못 잡고 Z_MIN 까지 감** | `LOAD_CONTACT` 를 낮춘다. `guard.baseline()` 이 편향됐을 수도 |
| **닿기도 전에 "접촉"** | `LOAD_CONTACT` 를 올린다. 이동 중 관성이 섞였을 수 있음 |
| **"카메라 프레임 지연/정지"** | 브리지 살아 있는지, USB 대역폭, `FRAME_STALL_MS` |
| `H.npy` 없음 | `python vision/03_calib_auto.py` — 카메라가 움직였으면 필수 |
| 로봇이 아무 데도 안 감 | `calib report` → H 나 작업영역이 미측정인지 |
| 계속 "범위 밖" / `OutOfReach` | 작업영역을 너무 좁게 훑었다 → `teach limits` 다시 |
| 서보(모터) 무응답 | `calib set <키> <값> --note "..."` 로 수동 진행 |
| 포트를 못 찾음 | `export SOARM_PORT=/dev/ttyACM0` (기본값도 ttyACM0) |
| 공을 못 찾음 / 깜빡임 (분류) | `lab_sample` 로 색 재측정, `min_area_ball` |
| 엉뚱한 걸 공으로 잡음 (분류) | `min_circularity` 를 올린다 |
| 파지 성공인데 실패로 뜸 (분류) | `teach grip` 으로 `grip_empty_frac` 재측정 |
| 자꾸 멈춤 (분류) | 지연 주입 슬라이더가 0 인지, `watchdog` 임계값 |

## 팀이 결정해야 할 것

**벤더 코드(`soarm_lab/`)를 수정했다.** 통합 과정에서 `driver_sdk.py`(기본 포트)와
`real.py`(`grip(frac)`)를 손댔다. 원래는 "벤더 코드는 안 고친다"였는데 깨졌다.
강사님이 새 버전을 배포하면 diff 가 지저분해지므로, **의도적으로 유지할지
어댑터로 걷어낼지** 팀이 명시적으로 정하는 게 좋다.

**분류 파이프라인(`sorting/pipeline.py`)을 남길지.** 지금은 회귀 기준선으로
남겨 뒀다(4단계 워치독·GUI·테스트 다수가 여기 물려 있음). 지우면 그 검증도
사라지므로, 지우려면 서보 루프 쪽으로 먼저 옮겨야 한다.
