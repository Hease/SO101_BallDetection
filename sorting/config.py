# -*- coding: utf-8 -*-
"""config.py — 튜닝 가능한 값을 전부 모아둔 곳.

여기 있는 숫자만 바꾸면 조명·카메라 높이·공 크기가 달라져도 대응된다.
다른 모듈은 상수를 자기 안에 두지 않고 반드시 여기서 가져다 쓴다.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")

H_FILE = os.path.join(DATA, "H.npy")                    # 픽셀→로봇 호모그래피
ZONE_FILE = os.path.join(DATA, "safety_zone.json")      # 안전 경계선 폴리곤
BASELINE_FILE = os.path.join(DATA, "depth_baseline.npy")  # 빈 테이블 depth
STATS_DIR = os.path.join(DATA, "stats")


# ── 색 정의 (CIELAB) ───────────────────────────────────────────────────────
#
# HSV 대신 Lab 을 쓰는 이유가 셋 있다.
#
#  1. 빨강이 쪼개지지 않는다. HSV 의 색상환에서 빨강은 0°와 180° 양끝에 걸쳐
#     있어 범위를 두 개로 나눠 OR 해야 했다. Lab 에는 그런 이음매가 없다.
#  2. 캘리브레이션이 클릭 한 번이다. 슬라이더 여섯 개(H/S/V 상·하한)를 맞추는
#     대신, 공을 클릭해 기준색 (a*, b*) 를 읽고 반경 하나만 정하면 된다.
#  3. 밝기가 분리된다. L(밝기)과 a*/b*(색기) 가 독립이라 그늘이 져도 색기는
#     거의 그대로다. HSV 의 S/V 는 조명에 같이 흔들린다.
#
# a* 는 녹(-)↔적(+), b* 는 청(-)↔황(+) 축이다. 아래 값은 shots 8장에서
# 실측한 중앙값이다(측정 방법은 README 의 '실측으로 맞춰야 하는 값' 참고).
#
#   ┌ b* (황)
#   │        ● 주황 구역
#   │   ● 빨간공(+34,+7)
#   ├──────────────── a* (적)
#   │
#   ● 파란공(+6,-29)
#
@dataclass(frozen=True)
class LabColor:
    """Lab 색공간의 기준색 하나. a*/b* 평면에서 radius 안이면 그 색으로 본다.

    a, b : OpenCV 8bit Lab 의 128 오프셋을 뺀 값 (즉 실제 a*, b*)
    radius : 허용 반경. 크면 관대해지고, 작으면 까다로워진다.
    l_min/l_max : 밝기 범위. 새까만 그림자와 하얗게 날아간 반사를 쳐낸다.
                  색 구분은 a*/b* 가 하므로 여기는 넉넉하게 둔다.
    draw : 화면 오버레이 색(BGR)
    """
    name: str
    a: float
    b: float
    radius: float
    draw: tuple[int, int, int]
    l_min: float = 25.0
    l_max: float = 245.0

    def distance_to(self, a: float, b: float) -> float:
        """기준색으로부터 a*/b* 평면 거리. 캘리브레이션 확인용."""
        return float(((a - self.a) ** 2 + (b - self.b) ** 2) ** 0.5)


# 실측 근거(shots 8장, 공 안쪽 픽셀의 중앙값):
#   빨간공  a*=+34 b*= +7   기준색에서 거리 0~2
#   파란공  a*= +6 b*=-29   기준색에서 거리 0.5~5.6
#   빨간 과자봉지(오검출 후보)  a*=+21 b*=+18 → 빨강 기준에서 16.3 떨어져 있다
# radius 12 는 공(≤2)을 넉넉히 품으면서 과자봉지(≥16.3)와 4 이상 떨어진 값이다.
RED = LabColor("red", a=+34.0, b=+7.0, radius=12.0, draw=(0, 0, 255))
BLUE = LabColor("blue", a=+6.0, b=-29.0, radius=16.0, draw=(255, 0, 0))

# ⚠ 주황 공급구역은 실측하지 못했다(촬영샷에 없음). 아래는 일반적인 주황의
#   대략값이므로 **실물 설치 후 반드시 다시 재야 한다**:
#       python -m sorting.lab_sample     ← 구역을 클릭하면 값을 알려준다
ORANGE = LabColor("orange", a=+25.0, b=+45.0, radius=20.0, draw=(0, 165, 255))

# 공 색 → 그 공을 놓을 bin 색. 여기에 줄을 추가하면 3색 이상으로 확장된다.
BALL_COLORS: dict[str, LabColor] = {"red": RED, "blue": BLUE}
BIN_COLORS: dict[str, LabColor] = {"red": RED, "blue": BLUE}
PLACE_MAP: dict[str, str] = {"red": "red", "blue": "blue"}

# 공이 처음 놓여 있는 공급 구역. None 이면 구역 제한 없이 화면 전체에서 공을 찾는다.
PICK_ZONE: LabColor | None = ORANGE


# ── 검출 임계값 ────────────────────────────────────────────────────────────
@dataclass
class VisionConfig:
    """블롭을 공/구역으로 가르는 기준.

    공과 구역은 '같은 색 검출'을 통과한 뒤 크기와 모양으로만 갈린다 —
    그래서 구역을 옮겨도 재캘리브레이션이 필요 없다.
    """
    min_area_ball: float = 300.0      # 이보다 작으면 노이즈
    max_area_ball: float = 6000.0     # 이보다 크면 공이 아니라 구역/배경
    # 4πA/P². shots 실측: 진짜 공 0.85~0.90, 과자봉지 등 잡동사니 ≤0.67.
    # 면적과 달리 카메라 높이가 바뀌어도 변하지 않아 더 믿을 만한 판별 기준이다.
    min_circularity: float = 0.75
    min_area_region: float = 8000.0   # 구역으로 인정할 최소 면적
    morph_kernel: int = 5             # 잔점 제거용 열림 연산 커널 크기
    blur_ksize: int = 5               # 0 이면 블러 생략


VISION = VisionConfig()


# ── 트래킹(검출 안정화) ────────────────────────────────────────────────────
@dataclass
class TrackingConfig:
    match_radius_px: float = 60.0   # 이 거리 안이면 같은 공으로 본다
    confirm_frames: int = 3         # 연속 이만큼 보여야 '확정'
    forget_frames: int = 8          # 연속 이만큼 안 보이면 폐기
    ema_alpha: float = 0.4          # 위치 평활화 (1.0 = 평활화 없음)


TRACKING = TrackingConfig()


# ── 안전(침입 감지) ────────────────────────────────────────────────────────
@dataclass
class SafetyConfig:
    intrusion_mm: int = 40          # 테이블보다 이만큼 높으면 물체로 본다
    # 감시 띠 면적 대비 몇 %가 솟으면 침입으로 볼지. 절대 픽셀 수로 잡으면
    # 깊이 해상도가 바뀔 때 감도가 조용히 달라진다(640x480↔320x240 은 4배 차이).
    min_band_fraction: float = 0.01
    min_pixels_floor: int = 200     # 아주 작은 프레임에서 노이즈로 트리거되지 않게
    enter_frames: int = 2           # 침입 판정에 필요한 연속 프레임
    clear_frames: int = 8           # 해제 판정에 필요한 연속 프레임(더 보수적)
    max_valid_mm: int = 2000        # 이보다 먼 depth 는 무효로 취급


SAFETY = SafetyConfig()


# ── 로봇 ───────────────────────────────────────────────────────────────────
@dataclass
class RobotConfig:
    port: str = "/dev/ttyACM0"
    ball_radius: float = 0.02       # 공 반지름(m) — 접근점 계산에 쓴다
    z_hover: float = 0.12           # 이동 중 손끝 높이(m)
    z_grasp: float = 0.005          # 파지 시 하강 높이(m)
    z_release: float = 0.08         # bin 위에서 놓는 높이(m)
    # 그리퍼는 fraction 으로 다룬다: 0=닫힘 .. 1=열림.
    # (팀 합의 — soarm_lab/real.py 의 grip(frac) 과 단위를 맞춘다. 각도(도)로 쓰면
    #  arm.go 의 grip= 의미와 섞이고, %로 쓰면 벤더 API 와 어긋난다.)
    grip_open: float = 1.0          # 활짝 열림
    grip_closed: float = 0.15       # 공을 물었을 때 — 공 크기에 맞춰 실측
    grip_empty_frac: float = 0.08   # 이보다 더 닫혔으면 아무것도 안 잡힌 것
    speed: int = 600                # 서보 속도(작을수록 느림, 0=최대)
    accel: int = 20                 # 가감속(작을수록 부드럽게)
    slow_speed: int = 250           # --speed slow 일 때
    slow_accel: int = 10
    settle_timeout: float = 4.0     # 도착 대기 최대 시간(초)
    settle_tol_deg: float = 3.0     # 목표각과 이만큼 가까우면 도착으로 본다
    max_retries: int = 2            # 파지 실패 시 재시도 횟수
    retry_z_drop: float = 0.004     # 재시도마다 이만큼 더 낮게 내려간다
    # 제외 반경(m) — "이 자리 근처는 같은 공으로 본다".
    # **공 지름(2 * ball_radius = 0.04m)보다 작아야 한다.** 크면 나란히 놓인
    # 두 공을 같은 공으로 오인해 하나를 건너뛴다.
    skip_radius_m: float = 0.03
    # 방금 처리한 자리를 이만큼 후보에서 뺀다. 트래커가 "이제 없다"를 확인할
    # 시간(forget_frames / 카메라 fps)보다 넉넉해야 같은 공을 다시 집지 않는다.
    handled_cooldown_s: float = 2.0
    home_pose_deg: tuple = (0.0, 30.0, -45.0, 0.0, 0.0)   # 안전 대기 자세


ROBOT = RobotConfig()


# ── 납땜 작업 ──────────────────────────────────────────────────────────────
@dataclass
class SolderConfig:
    """더미(비가열) 인두로 납땜 동작을 재현할 때의 값들.

    실제 가열 인두를 쓰지 않는 이유: 이 팔은 서보 기반이라 반복정밀도가 납땜에
    필요한 수준에 못 미치고, 인두 열이 그리퍼 플라스틱과 서보에 직접 닿는다.
    동작·정밀도·안전영역은 그대로 보여주면서 화상·화재 위험만 뺀 구성이다.
    """
    # 납땜점을 표시한 마커 색과, 보드로 볼 구역 색.
    point_color: str = "red"
    board_color: str = "blue"

    # **수평 이동은 반드시 이 높이에서.** 인두 끝을 보드에 끌면 패턴을 긁고
    # 부품을 밀어낸다. 이 값이 규칙을 코드로 강제하는 지점이다.
    z_safe: float = 0.06

    # 인두를 대는 높이. 실측 대상 — python -m sorting.teach heights
    z_solder: float = 0.005

    # 체류 시간(초). 납이 녹을 만큼. 중간에 사람이 들어오면 끊긴다.
    dwell_s: float = 2.0

    # 같은 점으로 볼 반경(m). 마커 간격보다 작아야 이웃 점을 건너뛰지 않는다.
    point_radius_m: float = 0.015


SOLDER = SolderConfig()


# ── GUI ────────────────────────────────────────────────────────────────────
@dataclass
class GuiConfig:
    twin_fps: int = 20              # 3D 트윈 렌더 주기
    twin_size: tuple[int, int] = (480, 360)
    camera_max_width: int = 640
    show_fps: bool = True


GUI = GuiConfig()
