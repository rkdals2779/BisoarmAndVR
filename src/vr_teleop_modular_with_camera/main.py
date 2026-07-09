"""
VR 텔레옵 실행 진입점
=====================
사용 예:
    python main.py --mode right   # 오른쪽 컨트롤러로 오른팔 로봇만 제어
    python main.py --mode left    # 왼쪽 컨트롤러로 왼팔 로봇만 제어
    python main.py --mode dual    # 양쪽 컨트롤러로 로봇 2대 동시 제어 (양손 텔레옵)

로봇 포트 / URDF 경로 등 실제 하드웨어 설정은 vr_teleop/config.py의
RIGHT_ARM_CONFIG / LEFT_ARM_CONFIG를 실제 환경에 맞게 수정하세요.
"""

import argparse
import atexit
import dataclasses
import signal

from vr_teleop.app import TeleopApp
from vr_teleop.config import TeleopConfig, get_arm_configs


def parse_args():
    parser = argparse.ArgumentParser(description="VR 텔레옵 컨트롤러")
    parser.add_argument(
        "--mode", choices=["left", "right", "dual"], default="left",
        help="left: 왼쪽 컨트롤러로 단일팔 / right: 오른쪽 컨트롤러로 단일팔 / dual: 양팔 동시 제어 (기본: right)",
    )
    parser.add_argument(
        "--hz", type=float, default=50.0,
        help="제어 루프 주파수 Hz (기본: 50)",
    )
    parser.add_argument(
        "--no-camera", action="store_true",
        help="왼팔에 카메라 pan/tilt 모터(id 7/8)가 물려 있지 않을 때 카메라 추종을 끕니다.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    arms = get_arm_configs(args.mode)
    if args.no_camera:
        # camera가 붙어 있는 ArmConfig만 골라서 camera=None으로 교체 (원본
        # 프리셋 자체는 건드리지 않도록 dataclasses.replace로 새 복사본 생성).
        arms = [dataclasses.replace(a, camera=None) if a.camera is not None else a for a in arms]

    teleop_config = TeleopConfig(control_hz=args.hz, arms=arms)

    app = TeleopApp(teleop_config)

    # 안전망 1: atexit. 여기서 등록해두면 정상 종료/Ctrl+C/아래에서 처리하는
    # 예외 등 어떤 경로로 파이썬 인터프리터가 종료되더라도 마지막으로 한 번 더
    # shutdown()이 실행됩니다. shutdown()은 이미 실행됐으면 아무 것도 안 하고
    # 바로 리턴하도록 만들어져 있으므로(app.py의 _shutdown_done), 여기서
    # 중복 호출돼도 안전합니다.
    atexit.register(app.shutdown)

    # 안전망 2: SIGTERM. Ctrl+C는 SIGINT라서 파이썬이 자동으로
    # KeyboardInterrupt 예외로 바꿔주지만, `kill <pid>`나 프로세스 매니저가
    # 기본으로 보내는 SIGTERM은 그렇지 않습니다 - 기본 동작은 try/finally를
    # 거치지 않고 즉시 프로세스를 종료시켜서 모터 토크가 켜진 채로 남습니다.
    # 그래서 SIGTERM도 KeyboardInterrupt로 변환해 기존 종료 경로(및 그
    # finally의 shutdown())를 그대로 타도록 만듭니다.
    def _handle_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _handle_sigterm)

    try:
        app.setup()
        app.run()
    except KeyboardInterrupt:
        # setup() 도중 Ctrl+C/SIGTERM이 들어온 경우: setup() 내부에서 이미
        # 연결된 만큼 토크를 해제하고 이 예외를 다시 던진 상태이므로, 여기서는
        # 보기 싫은 스택 트레이스 없이 조용히 끝내기만 하면 됩니다.
        # (run() 도중 들어온 Ctrl+C는 run() 내부에서 이미 메시지를 출력하고
        # shutdown()까지 마친 뒤 정상적으로 리턴하므로 이 분기까지 오지 않습니다.)
        print("\n[시스템] 초기화 단계에서 종료 신호를 받아 중단했습니다.")


if __name__ == "__main__":
    main()
