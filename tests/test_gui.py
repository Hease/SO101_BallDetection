# -*- coding: utf-8 -*-
"""GUI 를 화면 없이(offscreen) 띄워 배선이 실제로 도는지 확인한다.

여기서 잡고 싶은 건 "위젯이 예쁜가"가 아니라 **스레드 경계가 지켜지는가**다.
카메라·로봇이 각자 스레드에서 돌면서 메인 스레드에 신호를 제대로 꽂아주는지,
창을 닫을 때 스레드가 깨끗이 죽는지가 핵심이다.
"""
from __future__ import annotations

import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("SOARM_HEADLESS", "1")

pytest.importorskip("PySide6")

from PySide6.QtCore import QEventLoop, QTimer                       # noqa: E402
from PySide6.QtWidgets import QApplication                          # noqa: E402

from sorting.gui.views import CameraView, TwinView, bgr_to_pixmap   # noqa: E402
from sorting.replay import ReplaySource                              # noqa: E402
from sorting.vision import observe                                   # noqa: E402

# 이 파일은 전부 느린 테스트다 — Qt 이벤트루프와 스레드 기동을 실제로 기다린다.
# 빠른 피드백이 필요할 땐  pytest -m "not slow"
pytestmark = pytest.mark.slow

SHOTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "shots")


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def _spin(msec: int) -> None:
    """이벤트 루프를 msec 만큼 돌린다(시그널이 배달되도록)."""
    loop = QEventLoop()
    QTimer.singleShot(msec, loop.quit)
    loop.exec()


def test_bgr_to_pixmap_keeps_colors(app):
    """BGR→RGB 순서를 안 바꾸면 빨간 공이 파랗게 나온다."""
    bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    bgr[:, :] = (0, 0, 255)                      # BGR 빨강
    image = bgr_to_pixmap(bgr).toImage()
    color = image.pixelColor(2, 2)
    assert (color.red(), color.green(), color.blue()) == (255, 0, 0)


def test_camera_view_renders_detection(app):
    """검출 결과를 얹은 프레임이 실제로 픽스맵이 되는가."""
    import cv2
    img = cv2.imread(os.path.join(SHOTS, "shot_004.png"))
    view = CameraView()
    view.resize(640, 480)
    scene = observe(img)
    assert scene.balls, "이 사진에는 공이 있어야 한다"

    view.update_scene(img, scene, intruded=False, depth=None, fps=25.0,
                      latency_ms=12.3)
    assert view.pixmap() is not None and not view.pixmap().isNull()


def test_twin_renders_joint_angles(app):
    """3D 트윈이 관절각을 받아 실제 이미지를 만든다."""
    twin = TwinView()
    twin.resize(480, 360)
    if not twin.ensure_renderer():
        pytest.skip(f"이 환경에서 오프스크린 렌더 불가: {twin._error}")

    twin.update_joints([0, 30, -45, 0, 0, 50])
    first = twin.pixmap().toImage()
    twin.update_joints([60, 10, -20, 15, 0, 100])
    second = twin.pixmap().toImage()

    assert not first.isNull() and not second.isNull()
    assert first != second, "관절각을 바꿨는데 그림이 그대로다"
    twin.close_renderer()



def test_full_window_starts_and_stops_cleanly(app, tmp_path):
    """창을 띄우면 두 워커가 돌고, 닫으면 둘 다 깨끗이 멈춘다.

    스레드가 안 죽으면 프로그램이 종료되지 않고 매달린다 — 데모 끝나고
    창을 닫았는데 터미널이 안 돌아오는 그 상황이다.
    """
    from sorting.gui.main_window import MainWindow

    window = MainWindow(real=False, slow=False,
                        camera_source=ReplaySource(SHOTS, fps=25))
    window.resize(1024, 640)
    window.show()

    _spin(3000)
    assert window.camera_worker.isRunning(), "카메라 스레드가 안 돈다"
    assert window.camera_worker.fps > 0, "프레임이 안 들어온다"
    assert window.pipeline_worker.robot is not None, "로봇이 초기화되지 않았다"

    latest = window.camera_worker.latest()
    assert latest.scene is not None
    assert window.camera_view.pixmap() is not None

    window.close()
    _spin(500)
    assert not window.camera_worker.isRunning(), "카메라 스레드가 안 멈췄다"
    assert not window.pipeline_worker.isRunning(), "로봇 스레드가 안 멈췄다"



def test_estop_reaches_robot_without_waiting_for_worker(app):
    """E-STOP 은 워커 차례를 기다리지 않고 즉시 로봇에 닿아야 한다."""
    from sorting.gui.main_window import MainWindow

    window = MainWindow(real=False, slow=False,
                        camera_source=ReplaySource(SHOTS, fps=25))
    window.show()
    _spin(2500)
    assert window.pipeline_worker.robot is not None

    assert not window.pipeline_worker.robot.estopped
    window._on_estop()
    assert window.pipeline_worker.robot.estopped, "E-STOP 이 로봇에 즉시 닿지 않았다"

    window._on_reset()
    assert not window.pipeline_worker.robot.estopped

    window.close()
    _spin(500)
