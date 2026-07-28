# -*- coding: utf-8 -*-
"""views.py — 화면에 그림을 띄우는 두 위젯.

CameraView : 카메라 영상 + 검출 오버레이 + FPS. 여기서 bin 상자와 로봇좌표가
             실시간으로 따라 움직이는 게 보인다 — "위치가 바뀌어도 동작함"을
             말이 아니라 화면으로 증명하는 부분이다.
TwinView   : MuJoCo 를 창 없이 렌더해 Qt 안에 붙인다. 실물 모드에서도 같이
             움직이므로, 로봇이 지금 어떤 자세인지 옆에서 볼 수 있다.
"""
from __future__ import annotations

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel, QSizePolicy

from .. import config as cfg
from ..vision import draw_scene


def bgr_to_pixmap(bgr: np.ndarray) -> QPixmap:
    """OpenCV BGR 배열 → QPixmap.

    rgbSwapped() 를 쓰는 이유: OpenCV 는 BGR, Qt 는 RGB 순서다. 안 바꾸면
    빨간 공이 파랗게 나온다.
    """
    h, w = bgr.shape[:2]
    image = QImage(bgr.data, w, h, 3 * w, QImage.Format_BGR888)
    return QPixmap.fromImage(image.copy())


class ImagePanel(QLabel):
    """비율을 유지하며 이미지를 채우는 라벨."""

    def __init__(self, placeholder: str = "", parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background:#111; color:#888; border-radius:6px;")
        self.setText(placeholder)
        self._pixmap: QPixmap | None = None

    def show_pixmap(self, pixmap: QPixmap) -> None:
        self._pixmap = pixmap
        self._rescale()

    def resizeEvent(self, event):        # noqa: N802  (Qt 규약)
        super().resizeEvent(event)
        self._rescale()

    def _rescale(self) -> None:
        if self._pixmap is None:
            return
        self.setPixmap(self._pixmap.scaled(self.size(), Qt.KeepAspectRatio,
                                           Qt.SmoothTransformation))


class CameraView(ImagePanel):
    """라이브 영상 위에 검출 결과를 겹쳐 그린다."""

    def __init__(self, parent=None):
        super().__init__("카메라 대기 중…\n(shm_bridge 를 실행했는지 확인)", parent)
        self.safety = None
        self.mapper = None
        self.workspace = None       # 실측한 도달 한계. 있으면 화면에 그린다.

    def update_scene(self, rgb, scene, intruded: bool, depth=None,
                     fps: float = 0.0, latency_ms: float = 0.0) -> None:
        robot_xy = None
        if self.mapper is not None:
            # bin 중심의 로봇좌표를 같이 찍어준다 — 옮겼을 때 숫자가 바뀌는 게 보인다
            robot_xy = {name: self.mapper.to_robot(r.cx, r.cy)
                        for name, r in scene.regions.items()}

        frame = draw_scene(rgb, scene, robot_xy=robot_xy)

        # 실측한 작업영역을 그린다 — '한계'를 말이 아니라 눈으로 보이게 하는 부분.
        # 영역 밖의 공은 회색 X 로 표시해 왜 안 집는지 알 수 있게 한다.
        if self.workspace is not None and self.mapper is not None:
            self.workspace.draw(frame, self.mapper)
            self._mark_unreachable(frame, scene)

        if self.safety is not None:
            frame = self.safety.draw(frame, depth)

        if cfg.GUI.show_fps:
            self._draw_perf(frame, fps, latency_ms, len(scene.balls))
        if intruded:
            self._draw_intrusion_border(frame)

        self.show_pixmap(bgr_to_pixmap(frame))

    def _mark_unreachable(self, frame, scene) -> None:
        """작업영역 밖의 공에 회색 X — 검출은 됐지만 안 집는 이유를 보여준다."""
        import cv2
        for ball in scene.balls:
            x, y = self.mapper.to_robot(ball.u, ball.v)
            if self.workspace.check(x, y, self.workspace.z_min).ok:
                continue
            cv2.drawMarker(frame, (ball.u, ball.v), (150, 150, 150),
                           cv2.MARKER_TILTED_CROSS, ball.r * 2, 3)
            for color, thick in (((0, 0, 0), 3), ((180, 180, 180), 1)):
                cv2.putText(frame, "범위 밖", (ball.u - 24, ball.v + ball.r + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, thick, cv2.LINE_AA)

    @staticmethod
    def _draw_perf(frame, fps, latency_ms, ball_count) -> None:
        import cv2
        text = f"{fps:4.1f} FPS   검출 {latency_ms:4.1f}ms   공 {ball_count}"
        org = (frame.shape[1] - 320, 24)
        for color, thick in (((0, 0, 0), 3), ((200, 255, 200), 1)):
            cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        color, thick, cv2.LINE_AA)

    @staticmethod
    def _draw_intrusion_border(frame) -> None:
        import cv2
        h, w = frame.shape[:2]
        cv2.rectangle(frame, (0, 0), (w - 1, h - 1), (0, 0, 255), 8)


class TwinView(ImagePanel):
    """MuJoCo 3D 트윈. 오프스크린 렌더 → QPixmap.

    로봇 스레드의 물리와 완전히 분리된 자기 전용 MjModel/MjData 를 갖는다.
    관절각만 받아 그리므로 락을 다툴 일이 없고, 시뮬이든 실물이든 코드가 같다.
    """

    def __init__(self, parent=None):
        super().__init__("3D 트윈 준비 중…", parent)
        self._renderer = None
        self._model = None
        self._data = None
        self._error: str | None = None

    def ensure_renderer(self) -> bool:
        if self._renderer is not None:
            return True
        if self._error is not None:
            return False
        try:
            import mujoco
            from soarm_lab import SCENE
            w, h = cfg.GUI.twin_size
            self._model = mujoco.MjModel.from_xml_path(SCENE)
            self._data = mujoco.MjData(self._model)
            self._renderer = mujoco.Renderer(self._model, height=h, width=w)
            self._camera = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(self._camera)
            self._camera.distance = 0.9
            self._camera.azimuth = 135
            self._camera.elevation = -25
            self._camera.lookat[:] = [0.1, 0.0, 0.1]
            return True
        except Exception as exc:
            # 트윈은 있으면 좋은 것이지 필수가 아니다 — 실패해도 분류는 계속된다
            self._error = str(exc)
            self.setText(f"3D 트윈을 열 수 없습니다\n{exc}")
            return False

    def update_joints(self, angles_deg) -> None:
        if not self.ensure_renderer():
            return
        import mujoco

        n = min(len(angles_deg), self._model.nq)
        self._data.qpos[:n] = np.radians(np.asarray(angles_deg[:n], dtype=float))
        mujoco.mj_forward(self._model, self._data)
        self._renderer.update_scene(self._data, camera=self._camera)
        rgb = self._renderer.render()                       # (h, w, 3) RGB
        h, w = rgb.shape[:2]
        image = QImage(rgb.tobytes(), w, h, 3 * w, QImage.Format_RGB888)
        self.show_pixmap(QPixmap.fromImage(image))

    def close_renderer(self) -> None:
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None
