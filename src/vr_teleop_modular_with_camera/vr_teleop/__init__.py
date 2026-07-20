"""
VR 텔레옵 컨트롤러 패키지
=========================
모듈 구성:
    config.py       - 모든 설정값 (로봇/필터/IK/궤적/그리퍼/카메라, 좌/우 컨트롤러 프리셋)
    geometry.py      - OpenVR pose 행렬 <-> 위치/회전 변환 (하드웨어 의존성 없는 순수 수학)
    filters.py       - One Euro Filter (적응형 스무딩)
    trajectory.py     - 임계감쇠(critically damped) 관절 궤적 컨트롤러
    kinematics.py     - ikpy 기반 IK 체인 관리
    vr_interface.py    - OpenVR SDK 래퍼 (입력부)
    control.py        - 팔 1개에 대한 텔레옵 계산 파이프라인 (제어부)
    camera_head.py     - 카메라 pan/tilt 헤드 제어 (로봇 팔과 시리얼 포트 공유)
    output.py         - 로봇 명령 전송 + 콘솔 상태 출력 (출력부)
    app.py           - 전체 오케스트레이션 (단일팔/양팔 모드 + 카메라 공용 실행 루프)

사용법은 프로젝트 루트의 main.py와 README.md를 참고하세요.

주의: 이 파일 자체는 numpy만 있으면 임포트되도록 가볍게 유지했습니다.
      openvr / lerobot / ikpy / pyserial 같은 하드웨어 전용 패키지는 실제로
      그 기능을 쓰는 모듈(vr_interface.py, output.py, kinematics.py,
      camera_head.py)을 임포트하는 시점에만 필요합니다.
"""

from .config import (
	ArmConfig,
	CameraHeadConfig,
	ControllerRole,
	TeleopConfig,
	get_arm_configs,
)

__all__ = [
	'ArmConfig',
	'CameraHeadConfig',
	'ControllerRole',
	'TeleopConfig',
	'get_arm_configs',
]
