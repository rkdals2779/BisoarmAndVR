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
import dataclasses

from vr_teleop.app import TeleopApp
from vr_teleop.config import TeleopConfig, get_arm_configs


def parse_args():
    parser = argparse.ArgumentParser(description="VR 텔레옵 컨트롤러")
    parser.add_argument(
        "--mode", choices=["left", "right", "dual"], default="dual",
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
    app.setup()
    app.run()


if __name__ == "__main__":
    main()
