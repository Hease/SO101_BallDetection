# 납땜 모사 프로젝트 — 개발/진행 문서

> 팀 공유용 핸드오프 문서. 합류하는 팀원은 이 문서 → 실행 순서(§5) → mock 으로 먼저
> 돌려보기 순으로 온보딩하면 된다. (브리프 원문은 최상위 `PROJECT_BRIEF.md`)

---

## 0. 한 줄 요약 (현재 방향)

**인두기를 집게에 테이프로 강체 고정** → 탑다운 카메라로 **인두기 팁 마커**를
**타겟(드라이버 끝 마커)** 위로 **비주얼 서보**로 정렬 → 부하 감지될 때까지 하강 = **접촉**.
**인두기를 "집는"(파지) 동작은 하지 않는다.**

핵심 스크립트: [`solder/30_servo.py`](30_servo.py)

---

## 1. 이 방향으로 정해진 이유 (결정 이력)

| 단계 | 내용 |
|---|---|
| 원래 계획 | 인두기를 파지 → 팁을 타겟에 접촉 (브리프 §1) |
| 문제 | **파지 편차가 최대 난제** — 그립마다 인두기 자세가 달라져 팁 오차가 지배(§4.1). 원통형이라 파지·롤 슬립까지. |
| 결정 ① | **인두기를 집게에 강체 고정(테이프).** 파지 편차·슬립·원통 파지 문제 **통째로 소멸**. 남는 건 "팁을 타겟에 대기" 하나. |
| 결정 ② | 팁→타겟 정렬은 **탑다운 비주얼 서보(H 재사용)**. 두 마커를 로봇 평면 xy 로 바꿔 오차만큼 팔을 움직임. **팁 오프셋을 계산할 필요가 없다**(브리프 §4.2). 과제의 "비전 기반 제어" 요구도 충족. |

> 이전에 만든 파지 관련 스크립트(`10_teach`·`11_replay`·`20_locate`·`21_pick`)는
> **보관용**이다(§4). 현재 경로는 `30_servo` 하나로 수렴한다.

---

## 2. 하드웨어 / 환경

- **로봇**: SO-ARM101, Feetech STS3215. 관절 id 1~5(base·shoulder·elbow·wrist·roll), id6 집게.
  포트는 자주 바뀜 → 기본 `/dev/ttyACM0`, 필요시 바꿈(§3).
- **카메라**: 탑다운 HP60C(뎁스), `/dev/shm/hp60c_frames` 공유메모리 브리지로 읽음.
  손목 카메라는 현재 경로에선 **미사용**(필요해지면 붙임).
- **마커 부착(테이프/유성마커)**:
  - 인두기 **팁**에 마커(색 A) — 서보의 움직이는 쪽.
  - 드라이버 **끝**에 테이프(색 B) — 타겟(고정).
  - 두 색은 서로, 그리고 작업환경 색(분홍 마우스·노랑 테이프·주황 클램프)과 겹치지 않게.

---

## 3. 셋업 (처음 한 번)

```bash
# 0) 카메라 브리지 켜기 (탑다운 프레임 공급)
./hp60c-camera/scripts/start_bridge.sh

# 1) 포트 지정 — 사람마다 다르므로 개인 설정 파일(gitignore됨)에
echo '{"port": "/dev/ttyACM0"}' > data/local.json
#   또는 환경변수:  export SOARM_PORT=/dev/ttyACM0

# 2) (하드웨어 없이 로직만 볼 때) mock 모드 — 로봇/카메라를 print 로 대체
export SOARM_MOCK=1            # 둘 다 / SOARM_MOCK_ROBOT / SOARM_MOCK_CAMERA
```

- **캘리브레이션(H)**: 카메라가 움직였으면 `python vision/03_calib_auto.py` 로 `data/H.npy` 재생성.
  안 움직였으면 기존 H 그대로 사용.
- **마커색(Lab)**: `python vision/lab_tuner.py` 로 팁·타겟 두 색의 `LO/HI` 를 뽑아
  `30_servo.py` 상단 `TIP_LAB` / `TARGET_LAB` 에 입력. (클릭하면 그 점 Lab 값 출력)

---

## 4. 파일 구조

| 파일 | 역할 | 상태 |
|---|---|---|
| `solder/safety.py` | 안전 골격(긴급정지·부하가드·프레임워치독·관절/delta 한계) | 🟢 활성(공용) |
| `solder/hw.py` | 포트 해석(env·local.json·자동탐지) + 로봇/카메라 **print-mock** | 🟢 활성(공용) |
| **`solder/30_servo.py`** | **탑다운 비주얼 서보 + 접촉** — 현재 메인 | 🟢 활성 |
| `vision/lab_tuner.py` | Lab 색 튜너(마커색 잡기) | 🟢 활성 |
| `vision/03_calib_auto.py` | H 캘리브(픽셀→로봇평면) | 🟢 활성 |
| `solder/11_replay.py` | 부하기반 접촉정지 로직(참고) | 🟡 보관 |
| `solder/10_teach.py`·`20_locate.py`·`21_pick.py` | 파지 방식 잔재 | 🟡 보관(현재 경로 미사용) |

---

## 5. 실행 순서 (하드웨어)

```bash
# 항상 프로젝트 최상위에서 실행 (H 상대경로 때문)
# 1) 마커 검출/오차 확인 — 로봇 안 움직임
#    30_servo.py 의 MODE="preview" 로 두고:
python solder/30_servo.py
#    → 팁/타겟 십자표시 + 오차 화살표(팁→타겟)가 맞는지 확인

# 2) 실제 서보 — MODE="servo" 로 바꾼 뒤:
python solder/30_servo.py
#    → 정렬(폐루프) → 부하로 접촉까지 하강 → 후퇴.  정지: Enter / Ctrl-C
```

### 튜닝
- **발산**(팁이 타겟에서 멀어짐) → `GAIN` **부호를 뒤집는다**(H 방향 반대인 경우). 진동하면 `GAIN`↓.
- `STEP_CLAMP`(1스텝 최대 이동)·`Z_STEP`(하강 간격)으로 속도/안전.
- **접촉 임계** `safety.LOAD_CONTACT`: 하강 로그의 "부하상승" 값을 보고 공회전값과
  닿는 순간값 사이로. 접촉이 안 잡히면 낮추고, 오검출 정지가 잦으면 높인다.

---

## 6. 안전 / 과제 요구사항 매핑 (`safety.py`)

| 요구사항(브리프 §8) | 구현 |
|---|---|
| 긴급정지 버튼 | `EStop` — Enter/Ctrl-C → 즉시 토크오프. 모든 루프가 `est.check()` |
| 카메라 지연/끊김 보호 | `FrameWatchdog` — frame_id 가 N ms 이상 안 바뀌면 정지 |
| 로봇 제어 limit 데모 | `check_joint_limits`(한계초과 목표 거부) · `clamp_delta`(1스텝 이동 제한) |
| 접촉 감지 / 과부하 | `LoadGuard` — 부하 급증=접촉, 절대한계 초과=즉시정지 |
| 비전 기반 제어 시나리오 | `30_servo.py` 폐루프 자체 |
| 티칭/좌표 기반 제어 | H 캘리브 + 좌표 이동(보관 스크립트에 티칭도 있음) |

> 한계 상수(`MAX_STEP_DEG`·`LOAD_CONTACT`·`LOAD_ABORT`·`FRAME_STALL_MS`)는
> `safety.py` 상단에 모여 있다 — 현장에서 실측해 조정.

---

## 7. 현재 상태 / 다음 할 일

- ✅ 파이프라인 코드 완성(safety·hw·30_servo·lab_tuner). **mock 으로 서보 수렴·안전로직 검증됨.**
- ⬜ **실물 검증**: 마커 두 색 Lab 확정 / 서보 `GAIN` 부호·게인 / 접촉 `LOAD_CONTACT` 실측.
- ⬜ preview 로 두 마커·오차 방향 먼저 확인 → servo.
- ⬜ (여유 시) 손목 카메라 IBVS, 슬립 감지 등 브리프 상위 단계.

**낙오 팀원 서포트 포인트**: ① `lab_tuner` 로 마커색 잡기 ② preview 결과(검출·오차 화살표)
공유 ③ 접촉 부하값 로깅 — 여기부터 도우면 바로 붙는다.

---

## 8. 팀 협업 (git)

- **개인 설정은 커밋 안 함**: `data/local.json`(포트·mock), `H.npy`, `*.json` 티칭데이터는
  `.gitignore` 처리됨 → `git switch` 해도 안 섞임.
- **pyc 정리**: 각자 자기 브랜치에서 한 번 실행(이미 안 했다면):
  ```bash
  git ls-files | grep -E '__pycache__|\.pyc$' | xargs -r git rm --cached
  grep -qxF '__pycache__/' .gitignore || printf '\n__pycache__/\n*.pyc\n' >> .gitignore
  git add .gitignore && git commit -m "chore: pyc 추적 해제"
  ```
- **작업 브랜치**: 현재 방향은 `hease` 브랜치에서 진행. 합류 팀원은 아래로 받는다.

### 팀원 온보딩 (내려받고 적용)
```bash
git fetch origin
git switch hease            # 없으면: git switch -c hease origin/hease
# (가상환경/의존성은 기존 .venv 사용)
./hp60c-camera/scripts/start_bridge.sh
export SOARM_MOCK=1 && python solder/30_servo.py   # 하드웨어 없이 로직부터 확인
```
