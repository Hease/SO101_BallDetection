# -*- coding: utf-8 -*-
"""workspace.py — 로봇이 들어가도 되는 영역. **손으로 훑어서 실측한다.**

왜 따로 필요한가 — IK 를 직접 샘플링해 본 결과가 이유다.

    r(m)    0.25   0.30   0.35   0.40   0.44
    잔차     0.0    0.0    0.0    0.0    0.0 mm      ← 전부 "도달 가능"

기존 `Arm.REACH_TOL`(잔차 20mm) 방식은 `[2, 2, 2]` 같은 황당한 좌표에서만 걸린다.
운동학 모델은 반경 0.44m 까지 관절한계도 안 넘기고 풀어낸다. 하지만 그렇게 뻗은
자세는 서보 부하가 크고 정밀도가 떨어지며 카메라·bin 을 칠 수 있다.

    **운동학적 한계와 운용상 안전 한계는 다르다. 후자는 재야 안다.**

그래서 숫자를 코드에 적지 않는다. 토크를 끄고 사람이 **안전하다고 판단하는 범위의
테두리를 한 바퀴 돌면** 그 궤적으로 영역을 만든다. 책상·카메라·bin 배치가 불규칙해도
그 모양 그대로 담긴다.

새 의존성은 없다 — 이미 쓰는 OpenCV 로 충분하다:
`cv2.convexHull`(경계 만들기) · `cv2.pointPolygonTest`(안팎·여유 판정).

    python -m sorting.teach_limits     # 경계 훑어서 측정
    python -m sorting.workspace --demo # 하드웨어 없이 판정 확인
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from . import calib

CALIB_KEY = "workspace"


@dataclass(frozen=True)
class Verdict:
    """왜 안 되는지까지 말해주는 판정 결과. 화면에 그대로 띄울 수 있다."""
    ok: bool
    reason: str = ""
    margin_m: float = 0.0        # 경계까지 여유(m). 음수면 밖으로 나간 거리.

    def __bool__(self) -> bool:
        return self.ok


@dataclass(frozen=True)
class Workspace:
    """손으로 훑어 만든 작업영역. XY 는 폴리곤, Z 는 위아래 범위."""
    polygon_xy: list[tuple[float, float]]
    z_min: float
    z_max: float

    # ── 판정 ──────────────────────────────────────────────────────────────
    def _contour(self) -> np.ndarray:
        return np.asarray(self.polygon_xy, dtype=np.float32).reshape(-1, 1, 2)

    def margin_m(self, x: float, y: float) -> float:
        """경계까지의 거리(m). 안이면 양수, 밖이면 음수."""
        return float(cv2.pointPolygonTest(self._contour(), (float(x), float(y)), True))

    def check(self, x: float, y: float, z: float) -> Verdict:
        """이 좌표로 가도 되는가. 안 되면 이유를 붙여 돌려준다."""
        if len(self.polygon_xy) < 3:
            return Verdict(False, "작업영역이 아직 측정되지 않았습니다", 0.0)

        margin = self.margin_m(x, y)
        if margin < 0:
            return Verdict(False,
                           f"작업영역 밖 ({-margin * 1000:.0f}mm 벗어남)", margin)
        if z < self.z_min:
            return Verdict(False, f"너무 낮음 (z={z * 1000:.0f}mm, "
                                  f"최저 {self.z_min * 1000:.0f}mm)", margin)
        if z > self.z_max:
            return Verdict(False, f"너무 높음 (z={z * 1000:.0f}mm, "
                                  f"최고 {self.z_max * 1000:.0f}mm)", margin)
        return Verdict(True, "", margin)

    def contains(self, x: float, y: float, z: float) -> bool:
        return bool(self.check(x, y, z))

    # ── 만들기 ────────────────────────────────────────────────────────────
    @classmethod
    def from_trace(cls, points_xyz, shrink_m: float = 0.0) -> "Workspace":
        """손으로 훑은 궤적 → 볼록껍질 경계 + z 범위.

        볼록껍질을 쓰는 이유: 손으로 그린 궤적은 떨리고 되짚기도 해서 그대로
        폴리곤을 만들면 자기교차가 생긴다. 껍질을 취하면 항상 단순 볼록 도형이라
        안팎 판정이 명확하다. 대신 오목한 부분(예: 가운데 기둥)은 표현 못 한다 —
        그런 배치라면 그 부분을 피해 더 좁게 훑으면 된다.

        shrink_m > 0 이면 중심 쪽으로 그만큼 줄여 안전여유를 둔다.
        """
        pts = np.asarray([(p[0], p[1]) for p in points_xyz], dtype=np.float32)
        if len(pts) < 3:
            raise ValueError("경계를 만들려면 점이 3개 이상 필요합니다")

        hull = cv2.convexHull(pts.reshape(-1, 1, 2)).reshape(-1, 2)
        if shrink_m > 0:
            center = hull.mean(axis=0)
            out = []
            for p in hull:
                d = p - center
                dist = float(np.hypot(*d))
                scale = max(0.0, (dist - shrink_m)) / dist if dist > 1e-9 else 0.0
                out.append(center + d * scale)
            hull = np.asarray(out, dtype=np.float32)

        zs = [float(p[2]) for p in points_xyz]
        return cls([(float(a), float(b)) for a, b in hull], min(zs), max(zs))

    # ── 저장·로드 (실측값 저장소를 통해) ──────────────────────────────────
    def to_dict(self) -> dict:
        return {"polygon_xy": [list(p) for p in self.polygon_xy],
                "z_min": self.z_min, "z_max": self.z_max}

    @classmethod
    def from_dict(cls, data: dict) -> "Workspace":
        return cls([tuple(p) for p in data["polygon_xy"]],
                   float(data["z_min"]), float(data["z_max"]))

    def save(self, note: str = "", manual: bool = False) -> None:
        setter = calib.STORE.set_manual if manual else calib.STORE.set_measured
        setter(CALIB_KEY, self.to_dict(), note)

    @classmethod
    def load(cls) -> "Workspace | None":
        """측정된 영역. 아직 안 쟀으면 None — **기본값을 지어내지 않는다.**

        여기서 그럴듯한 기본값을 돌려주면 "한계가 있는 척"하게 된다.
        측정 전에는 없다고 정직하게 말하고, 부르는 쪽이 경고하도록 한다.
        """
        raw = calib.STORE.get(CALIB_KEY, None)
        if raw.value is None:
            return None
        try:
            return cls.from_dict(raw.value)
        except Exception as exc:
            print(f"[workspace] 저장된 영역을 읽지 못했습니다: {exc}")
            return None

    # ── 화면 표시 ─────────────────────────────────────────────────────────
    def to_pixels(self, mapper) -> np.ndarray:
        """로봇좌표 경계 → 카메라 픽셀. `Mapper.to_pixel` 을 그대로 쓴다."""
        return np.array([mapper.to_pixel(x, y) for x, y in self.polygon_xy],
                        dtype=np.int32)

    def draw(self, bgr, mapper, label: bool = True):
        """카메라 화면에 경계선을 그린다. 데모에서 '한계'를 눈으로 보이게 하는 부분."""
        pts = self.to_pixels(mapper).reshape(-1, 1, 2)
        cv2.polylines(bgr, [pts], True, (0, 200, 255), 2)
        if label:
            top = pts.reshape(-1, 2)
            anchor = tuple(top[int(np.argmin(top[:, 1]))])
            text = f"작업영역 (z {self.z_min * 100:.0f}~{self.z_max * 100:.0f}cm)"
            for color, thick in (((0, 0, 0), 3), ((0, 200, 255), 1)):
                cv2.putText(bgr, text, (int(anchor[0]) - 60, max(int(anchor[1]) - 10, 16)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thick, cv2.LINE_AA)
        return bgr

    @property
    def summary(self) -> str:
        pts = np.asarray(self.polygon_xy)
        r = np.hypot(pts[:, 0], pts[:, 1])
        return (f"꼭짓점 {len(pts)}개 · 베이스에서 {r.min():.3f}~{r.max():.3f}m · "
                f"z {self.z_min:.3f}~{self.z_max:.3f}m")


# ── 데모 (하드웨어 없이 판정 확인) ─────────────────────────────────────────
def _demo() -> int:
    """저장된 영역이 있으면 그걸로, 없으면 가상의 영역으로 판정을 보여준다."""
    ws = Workspace.load()
    if ws is None:
        print("측정된 작업영역이 없습니다 — 가상 영역으로 판정만 시연합니다.")
        print("(실제 측정:  python -m sorting.teach_limits)\n")
        trace = [(0.13, -0.14, 0.02), (0.28, -0.16, 0.03), (0.30, 0.0, 0.20),
                 (0.28, 0.16, 0.05), (0.13, 0.14, 0.02), (0.11, 0.0, 0.01)]
        ws = Workspace.from_trace(trace)
    else:
        print(f"측정된 작업영역: {ws.summary}\n")

    print(f"영역: {ws.summary}\n")
    print(f"{'좌표 (x, y, z)':<26} {'판정':<6} 이유")
    print("-" * 74)
    for xyz in [(0.20, 0.00, 0.10), (0.25, 0.05, 0.10), (0.44, 0.00, 0.10),
                (0.35, 0.00, 0.10), (0.20, 0.00, 0.40), (0.20, 0.00, -0.05),
                (0.05, 0.00, 0.05)]:
        v = ws.check(*xyz)
        mark = "통과" if v.ok else "거절"
        print(f"  ({xyz[0]:+.2f}, {xyz[1]:+.2f}, {xyz[2]:+.2f})        "
              f"{mark:<6} {v.reason or f'여유 {v.margin_m * 1000:.0f}mm'}")

    print("\n운동학 IK 는 반경 0.44m 까지 잔차 0mm 로 풀어낸다 — 즉 IK 만 믿으면")
    print("위 0.44m 좌표가 그대로 통과한다. 실측한 영역이 그걸 막는다.")
    return 0


if __name__ == "__main__":
    import sys
    raise SystemExit(_demo() if "--demo" in sys.argv else _demo())
