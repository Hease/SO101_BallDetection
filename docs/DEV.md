# 개발 문서 (내부 공유용)

## 이 시스템이 무엇을 하는가

카메라로 빨강·파랑 공을 찾아 색에 맞는 구역에 담는다. 구역을 옮겨도 따라간다.
그 위에 **안전장치 세 겹**(사람 감지·도달 한계·딜레이 보호)과 **실측 기반
캘리브레이션**을 얹었다.

## 계층 구조

Intel RVC 참고 구조와 대응된다(발표에서 이 대응을 쓰면 설명이 쉽다).

```
   RVC                          우리 코드
 ─────────────────────────────────────────────────────────
   Sensing                      hp60c-camera (RGB-D)
   Vision                       vision.py · tracking.py
   (좌표 변환)                   mapping.py   ← 4점 호모그래피
   Motion Control Interface     robot.py     ← RVC 의 motion_controller_interface 자리
   Use Case / Task              pipeline.py  ← 상태머신
   Manipulation                 soarm_lab (IK · 시뮬/실물 백엔드, 수정 없음)
 ─────────────────────────────────────────────────────────
   안전은 계층을 가로지른다:  safety.py · workspace.py · watchdog.py
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
| z_grasp / z_hover / z_release | `teach heights` | 팔끝을 책상·bin 에 대고 Enter |
| 그리퍼 열림/닫힘/빈손기준 | `teach grip` | 실제로 공을 물려보고 읽음 |
| 홈 자세 | `teach home` | 안전한 대기 자세를 손으로 잡고 Enter |
| bin 좌표 등 | `teach add <이름>` | 팔끝을 그 지점에 대고 Enter |
| Lab 기준색 | `lab_sample` | 대상을 클릭 → 붙여넣을 한 줄이 나옴 |
| 픽셀→로봇 변환 H | `calibrate` | 공을 네 곳에 옮기며 팔끝을 대줌 |
| 안전 경계선·테이블 기준면 | `setup_zone` | 작업영역 클릭 + 빈 테이블 촬영 |
| 아무거나 (수동) | `calib set <키> <값> --note "사유"` | 자동이 막혔을 때 |

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
pytest -m "not slow"    # 3초 — 개발 중에는 항상 이것
pytest                  # 90초 — 커밋 전 한 번
```

**빠른 레인이 있는 이유**: 요구사항이 발표 3시간 전에 바뀐다. 5초 피드백과
100초 피드백은 완전히 다른 작업이 된다. `tests/fakes.py` 의 `FakeRobot` 은
물리 없이 즉시 이동하므로 파이프라인 판단을 밀리초에 검증한다.

| 파일 | 지키는 것 |
|---|---|
| `test_vision.py` | 촬영샷 8장의 공 개수, 과자봉지 오검출 |
| `test_pipeline_fast.py` | 순서·분기·재시도 (물리 없이, 빠름) |
| `test_pipeline.py` | 실제 IK·물리까지 (느림, `slow` 마커) |
| `test_safety_integration.py` | **배선** — 모듈이 실제로 물려 있는가 |
| `test_workspace.py` | 안팎 판정, IK 가 통과시키는 0.44m 거절 |
| `test_watchdog.py` | 3단계 전이·히스테리시스·회복 |
| `test_calib.py` | 출처 추적, 수동 덮어쓰기, 파일 깨져도 안 죽음 |
| `test_teach.py` | 훑기→영역, 서보 무응답 시 수동 경로 |

---

## ⚡ 새 요구사항이 오면 어디를 고치나

발표 3시간 전 대응용. **요구 유형 → 고칠 파일 → 확인 명령** 순서로 본다.

| 요구 유형 | 고칠 곳 | 확인 |
|---|---|---|
| **색 추가** (초록 공 등) | `config.py` 의 `BALL_COLORS`/`BIN_COLORS`/`PLACE_MAP` 에 한 줄씩 + `lab_sample` 로 색 측정 | `pytest -m "not slow"` |
| **분류 규칙 변경** (크기별·모양별 등) | `pipeline._choose_ball` (무엇을 먼저) / `_run_cycle` 의 `bin_color` 결정 | `test_pipeline_fast.py` |
| **동작 순서 변경** | `robot.pick` / `robot.place` 의 웨이포인트 | `test_pipeline_fast.py::test_pick_happens_before_place` |
| **새 안전 규칙** | 공간이면 `workspace.py`, 시간이면 `watchdog.py` | `test_safety_integration.py` |
| **검출 대상 추가** (박스·마커 등) | `vision.detect_region` 재사용 + `Scene` 에 필드 | `test_vision.py` |
| **속도·정밀도 요구** | `config.RobotConfig` 의 speed/accel/settle_tol | 실물 확인 |
| **새 좌표를 손으로 지정** | `teach add <이름>` → `PoseLibrary` 에서 읽어 씀 | `teach list` |
| **UI 표시 항목 추가** | `gui/panel.py` 의 `_build_status` | GUI 실행 |

**작업 순서 권장**: ① `pytest -m "not slow"` 로 현재 초록 확인 → ② 테스트 먼저
한 줄 추가 → ③ 구현 → ④ 빠른 레인 → ⑤ 전체 레인 → ⑥ 커밋.

**변경 영향범위 판정**은 C 트랙(문서·테스트 담당)이 맡는다. 위 표에서 해당
행을 찾고, 그 행의 확인 명령이 초록이면 나머지는 건드리지 않은 것이다.

## 자주 겪는 문제

| 증상 | 볼 곳 |
|---|---|
| 공을 못 찾음 / 깜빡임 | `lab_sample` 로 색 재측정, `min_area_ball` |
| 엉뚱한 걸 공으로 잡음 | `min_circularity` 를 올린다 |
| 로봇이 아무 데도 안 감 | `calib report` → H 나 작업영역이 미측정인지 |
| 계속 "범위 밖" | 작업영역을 너무 좁게 훑었다 → `teach limits` 다시 |
| 자꾸 멈춤 | 지연 주입 슬라이더가 0 인지, `watchdog` 임계값 |
| 파지 성공인데 실패로 뜸 | `teach grip` 으로 `grip_empty_pct` 재측정 |
| 서보 무응답 | `calib set <키> <값> --note "..."` 로 수동 진행 |
