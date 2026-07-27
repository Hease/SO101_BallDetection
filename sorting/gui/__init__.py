# -*- coding: utf-8 -*-
"""sorting.gui — PySide6 데스크톱 화면.

구조는 "메인 스레드는 그리기만 한다" 하나로 요약된다:

    [CameraWorker] 카메라 읽기 → 검출 → 트래킹 → 안전감시   (스레드 1)
          │ sceneReady
          ├──────────────→ [메인] 영상·오버레이·통계 그리기
          │
    [PipelineWorker] 상태머신 + 로봇 시리얼                  (스레드 2)
          │ jointsChanged / stateChanged
          └──────────────→ [메인] 3D 트윈 렌더

카메라도 로봇도 블로킹 I/O 라서, 하나라도 메인 스레드에 두면 창이 얼어붙는다.
"""
