# -*- coding: utf-8 -*-
"""main.py — 색 분류 스테이션 진입점.

    python sorting_main.py --sim                     # 시뮬 + GUI (로봇·카메라 없이도 뜬다)
    python sorting_main.py --sim --replay shots   # 저장된 사진을 카메라 대신 사용
    python sorting_main.py --real --slow             # 실물, 저속 (처음엔 반드시 이걸로)

⚠ --real 은 팔이 실제로 움직입니다. 작업면에 손을 넣지 마세요.
   E-STOP 버튼이 듣는지를 빈 작업면에서 먼저 확인하고 공을 올리세요.
"""
from __future__ import annotations

import argparse
import os
import sys


def parse_args(argv):
    ap = argparse.ArgumentParser(
        description="SO-ARM101 빨강·파랑 공 분류 스테이션",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true", default=True,
                      help="시뮬레이션 (기본)")
    mode.add_argument("--real", action="store_true",
                      help="실물 로봇 — 팔이 실제로 움직입니다")
    ap.add_argument("--slow", action="store_true",
                    help="저속 동작. 실물 첫 실행에는 반드시 켜세요")
    ap.add_argument("--replay", metavar="경로",
                    help="카메라 대신 저장된 사진 사용 (폴더 또는 glob)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    real = bool(args.real)

    if not real:
        # LiveSim 이 자기 창을 띄우지 않게 한다 — 3D 는 Qt 안에서 그린다
        os.environ.setdefault("SOARM_HEADLESS", "1")

    from PySide6.QtWidgets import QApplication

    from sorting.gui.main_window import MainWindow
    from sorting.replay import ReplaySource

    camera_source = ReplaySource(args.replay) if args.replay else None

    if real:
        print("\n⚠ 실물 모드입니다. 작업면에 손을 넣지 마세요.")
        print("  E-STOP 버튼이 듣는지 빈 작업면에서 먼저 확인하세요.\n")

    app = QApplication(sys.argv[:1])
    window = MainWindow(real=real, slow=args.slow,
                        camera_source=camera_source)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
