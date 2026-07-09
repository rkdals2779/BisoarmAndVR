"""
설정 모듈
=========
로봇/필터/IK/궤적/그리퍼 등 모든 튜닝 파라미터를 한 곳에 모아둔 모듈입니다.
값을 바꾸고 싶으면 항상 이 파일만 보면 됩니다 (다른 모듈에는 숫자 상수가
없습니다).

핵심 아이디어: "팔 하나(=VR 컨트롤러 한쪽 + 팔로워 로봇 한 대)"의 모든
설정을 ArmConfig 하나에 담습니다. 그래서

    - 오른쪽 컨트롤러로 로봇 1대만 제어  -> arms=[RIGHT_ARM_CONFIG]
    - 왼쪽 컨트롤러로 로봇 1대만 제어    -> arms=[LEFT_ARM_CONFIG]
    - 양손으로 로봇 2대 동시 제어(양팔) -> arms=[LEFT_ARM_CONFIG, RIGHT_ARM_CONFIG]

처럼 리스트에 ArmConfig를 몇 개 넣느냐로 단일팔/양팔 모드가 결정됩니다.
control.py/app.py는 팔이 1개든 2개든 완전히 동일한 코드로 동작합니다.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np


class ControllerRole(Enum):
    """VR 컨트롤러가 왼손용인지 오른손용인지 (openvr의 TrackedControllerRole과 대응)"""
    LEFT = "left"
    RIGHT = "right"


# ============================================================
# 세부 파라미터 그룹
# ============================================================
@dataclass
class OneEuroFilterConfig:
    """
    min_cutoff: 손이 멈춰있을 때 기본 스무딩 강도 (낮을수록 부드러움)
    beta:       손이 빠르게 움직일 때 필터를 얼마나 "풀어주는지" (높을수록 반응 빠름/지연 적음)
    """
    min_cutoff: float
    beta: float
    d_cutoff: float = 1.0


@dataclass
class TrajectoryConfig:
    """임계감쇠(critically damped) 관절 궤적 컨트롤러 파라미터"""
    kp: float = 45.0                    # 반응성 (높을수록 빠르지만 서보가 못 따라가면 진동 위험)
    max_vel_deg_s: float = 120.0        # deg/s - 실제 로봇에서 보수적으로 시작해서 올릴 것
    max_acc_deg_s2: float = 600.0       # deg/s^2


@dataclass
class WristMappingConfig:
    """
    VR 컨트롤러 pitch/roll -> 손목 관절(wrist_flex/wrist_roll) 직접 매핑 설정.
    IK를 거치지 않고 물리적으로 대응되는 관절에 1:1로 직접 매핑합니다
    (이유는 control.py 상단 docstring 참고).

    반대로 움직이면 해당 *_sign 값을 -1.0으로 뒤집으세요.
    """
    wrist_flex_key: str = "wrist_flex.pos"
    wrist_roll_key: str = "wrist_roll.pos"
    pitch_scale: float = 1.0            # 컨트롤러 pitch -> wrist_flex 배율 (1.0 = 1:1)
    roll_scale: float = 1.0             # 컨트롤러 roll -> wrist_roll 배율 (1.0 = 1:1)
    pitch_sign: float = -1.0            # wrist_flex 반대로 움직이면 부호를 다시 뒤집으세요
    roll_sign: float = 1.0              # wrist_roll 반대로 움직이면 -1.0
    filter: OneEuroFilterConfig = field(
        default_factory=lambda: OneEuroFilterConfig(min_cutoff=1.0, beta=0.3, d_cutoff=1.0)
    )


@dataclass
class GripperConfig:
    """그리퍼는 IK/회전 매핑과 완전히 분리해서 VR 트리거로 직접 제어합니다."""
    key: str = "gripper.pos"
    open_deg: float = 90.0
    close_deg: float = 0.0   # 실제 그리퍼 가동 범위를 확인 후 조정하세요


@dataclass
class CameraHeadConfig:
    """
    HMD(헤드셋) 방향을 따라가는 카메라 pan/tilt 헤드 설정.

    중요: 이 카메라 모터(기본 id 7/8)는 로봇 팔과 "물리적으로 같은 시리얼
    버스"(같은 USB-TTL 어댑터에 데이지체인으로 연결된 하나의 /dev/ttyACM*)에
    달려 있다고 가정합니다. 그래서 camera_head.py는 별도의 serial.Serial을
    새로 열지 않고, 이 설정이 달린 ArmConfig(RobotOutput)가 이미 열어놓은
    시리얼 연결을 그대로 재사용합니다. 두 개의 독립된 Serial 연결이 같은
    버스에 동시에 패킷을 쓰면 충돌해서 양쪽 다 통신 오류가 나기 때문입니다.
    """
    pan_motor_id: int = 7
    tilt_motor_id: int = 8

    pan_home_deg: float = 180.0
    tilt_home_deg: float = 220.0

    pan_min_deg: float = 90.0
    pan_max_deg: float = 270.0
    tilt_min_deg: float = 110.0
    tilt_max_deg: float = 250.0

    # HMD -> 모터 각도 매핑 및 방향 (반대로 돌면 부호를 -1.0으로 수정)
    yaw_scale: float = 1.0
    pitch_scale: float = -1.0
    yaw_sign: float = -1.0
    pitch_sign: float = 1.0

    rot_filter: OneEuroFilterConfig = field(
        default_factory=lambda: OneEuroFilterConfig(min_cutoff=1.0, beta=0.3, d_cutoff=1.0)
    )
    trajectory: TrajectoryConfig = field(
        default_factory=lambda: TrajectoryConfig(kp=45.0, max_vel_deg_s=150.0, max_acc_deg_s2=600.0)
    )


@dataclass
class IKConfig:
    """
    urdf_path: 이 팔(로봇)이 사용할 URDF 파일 경로
    arm_joint_keys: IK 체인에서 위치 IK에 사용되는 활성 관절 순서라고 "가정"한
        목록입니다. 표준 SO-101 URDF 기준 가정이며, 실행 시 kinematics.py의
        진단 출력으로 반드시 검증하세요 (개수가 안 맞으면 assert로 즉시 실패합니다).
    skip_threshold_m: 이 값보다 적게 움직이면 위치 IK 재계산을 생략합니다 (성능 최적화).
    """
    urdf_path: str
    skip_threshold_m: float = 0.001
    arm_joint_keys: List[str] = field(default_factory=lambda: [
        "shoulder_pan.pos",
        "shoulder_lift.pos",
        "elbow_flex.pos",
        "wrist_flex.pos",
        "wrist_roll.pos",
    ])


@dataclass
class ArmConfig:
    """VR 컨트롤러 하나 + 팔로워 로봇 하나로 구성된 '팔 한쪽'의 전체 설정"""
    name: str                            # 로그/상태 표시에 쓰일 이름 (예: "right_arm")
    controller_role: ControllerRole      # 이 팔이 어느 손 컨트롤러를 따라갈지
    robot_id: str
    robot_port: str
    home_pos: np.ndarray                 # 컨트롤러 델타가 0일 때 로봇 TCP 목표 위치 (m)
    ik: IKConfig

    scale_factor: float = 0.8            # VR 이동량 -> 로봇 이동량 배율
    # 로봇이 반대쪽(거울 대칭)으로 장착되어 특정 축이 반대로 움직이면
    # 여기서 -1.0으로 뒤집으세요. (x, y, z) 순서, 기본은 뒤집지 않음.
    axis_signs: Tuple[float, float, float] = (1.0, 1.0, 1.0)

    position_filter: OneEuroFilterConfig = field(
        default_factory=lambda: OneEuroFilterConfig(min_cutoff=0.8, beta=1.0, d_cutoff=1.0)
    )
    wrist: WristMappingConfig = field(default_factory=WristMappingConfig)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    trajectory: TrajectoryConfig = field(default_factory=TrajectoryConfig)

    # 이 팔의 로봇과 같은 시리얼 포트(같은 물리적 버스)에 카메라 pan/tilt
    # 모터가 함께 달려 있는 경우에만 설정하세요 (예: 왼팔 = /dev/ttyACM0에
    # 카메라 모터 id 7/8도 데이지체인으로 연결된 경우). None이면 카메라
    # 기능은 아예 동작하지 않습니다.
    camera: Optional[CameraHeadConfig] = None


@dataclass
class TeleopConfig:
    """실행할 팔들의 목록 + 공용 제어 주기"""
    control_hz: float = 50.0
    arms: List[ArmConfig] = field(default_factory=list)

    @property
    def dt_nominal(self) -> float:
        return 1.0 / self.control_hz


# ============================================================
# 프리셋: 로봇 2대(오른쪽/왼쪽) 기본 설정
# ------------------------------------------------------------
# 실제 환경에 맞게 robot_port / urdf_path / robot_id 를 반드시 수정하세요.
# 양팔 모드를 쓸 게 아니라면 LEFT_ARM_CONFIG는 그대로 둬도 무방합니다
# (get_arm_configs("right")를 쓰면 참조되지 않습니다).
# ============================================================
URDF_PATH_DEFAULT = "/home/roboseasy/shin_ws/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"

RIGHT_ARM_CONFIG = ArmConfig(
    name="right_arm",
    controller_role=ControllerRole.RIGHT,
    robot_id="skm_right_follower",
    robot_port="/dev/ttyACM1",
    home_pos=np.array([0.2, 0.0, 0.1]),
    ik=IKConfig(urdf_path=URDF_PATH_DEFAULT),
)

LEFT_ARM_CONFIG = ArmConfig(
    name="left_arm",
    controller_role=ControllerRole.LEFT,
    robot_id="skm_left_follower",
    robot_port="/dev/ttyACM0",
    home_pos=np.array([0.2, 0.0, 0.1]),
    ik=IKConfig(urdf_path=URDF_PATH_DEFAULT),
    # 왼팔 로봇이 오른팔과 물리적으로 거울 대칭 장착된 경우가 많습니다.
    # 실제로 움직여보고 방향이 반대인 축이 있으면 axis_signs / wrist의
    # pitch_sign, roll_sign을 -1.0으로 조정하세요. 아래는 기본값(오른팔과 동일)입니다.

    # 카메라 pan/tilt 모터(id 7/8)가 왼팔과 같은 /dev/ttyACM0 버스에
    # 데이지체인으로 물려 있으므로 여기에 붙입니다. camera_head.py가
    # 이 팔의 RobotOutput이 이미 열어놓은 시리얼 연결을 그대로 재사용해서
    # 카메라를 제어합니다 (포트를 두 번 여는 게 원래 오류의 원인이었습니다).
    camera=CameraHeadConfig(),
)


def get_arm_configs(mode: str) -> List[ArmConfig]:
    """
    mode:
        "right" -> 오른쪽 컨트롤러 1개로 오른팔 로봇만 제어 (단일팔)
        "left"  -> 왼쪽 컨트롤러 1개로 왼팔 로봇만 제어 (단일팔)
        "dual"  -> 양쪽 컨트롤러로 로봇 2대를 동시에 제어 (양손 텔레옵)
    """
    mode = mode.lower()
    if mode == "right":
        return [RIGHT_ARM_CONFIG]
    if mode == "left":
        return [LEFT_ARM_CONFIG]
    if mode == "dual":
        return [LEFT_ARM_CONFIG, RIGHT_ARM_CONFIG]
    raise ValueError(f"알 수 없는 모드: {mode!r} ('left'/'right'/'dual' 중 하나여야 합니다)")
