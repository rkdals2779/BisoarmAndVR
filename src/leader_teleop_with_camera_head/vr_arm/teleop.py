"""VR 컨트롤러로 팔로워암을 조종하는 lerobot Teleoperator 구현.

기존 vr_teleop 패키지의 계산 파이프라인(One Euro Filter -> 위치 IK ->
손목 직접 매핑 -> 임계감쇠 궤적)을 그대로 재사용해서, lerobot의
Teleoperator 인터페이스(get_action이 관절 목표각 딕셔너리 반환)로
감싼다. 리더암(bi_so_leader)과 같은 액션 키를 내보내므로 teleoperate/
record 파이프라인에 그대로 끼울 수 있다.

주의: 하드웨어(시리얼)는 전혀 다루지 않는다 - 로봇 구동은 lerobot의
robot 객체가 담당하고, 이 클래스는 VR pose -> 관절 목표각 계산만 한다.

의존성: openvr, scipy, ikpy, numpy (+ SteamVR/ALVR 실행 중이어야 함).
"""

import dataclasses
import logging
import sys
import time
from pathlib import Path
from typing import Any, Final

from lerobot.teleoperators.teleoperator import Teleoperator

from .config import BiVrLeaderConfig

logger = logging.getLogger(__name__)

# vr_teleop 패키지 위치: {레포루트}/src/vr_teleop_modular_with_camera
_VR_TELEOP_PARENT_DIR: Final[Path] = (
	Path(__file__).resolve().parents[2] / 'vr_teleop_modular_with_camera'
)
if str(_VR_TELEOP_PARENT_DIR) not in sys.path:
	sys.path.insert(0, str(_VR_TELEOP_PARENT_DIR))

try:
	from vr_teleop.config import (
		ArmConfig,
		GripperConfig,
		get_arm_configs,
	)
	from vr_teleop.control import ArmTeleopController
	from vr_teleop.kinematics import RobotKinematics
	from vr_teleop.vr_interface import VRSystem
except ImportError as error:
	raise RuntimeError(
		'bi_vr_leader에는 openvr/scipy/ikpy/numpy가 필요합니다. '
		f'실행 환경에 설치 후 다시 실행하세요. (원본 오류: {error})'
	) from error


class _VrArmRuntime:
	"""팔 한쪽의 VR 텔레옵 런타임 묶음 (내부용).

	Attributes:
		arm_config: vr_teleop ArmConfig (프리셋 + CLI 오버라이드 적용).
		controller: VR pose -> 관절 목표각 계산기.
		prefix: 액션 키 접두사 ('left_'/'right_', 단일팔이면 '').
		hold_action: pose 유실/캘리브레이션 중에 보낼 유지 액션.
	"""

	def __init__(self, arm_config: 'ArmConfig', prefix: str) -> None:
		self.arm_config = arm_config
		self.prefix = prefix
		kinematics = RobotKinematics(
			arm_config.ik,
			arm_config.wrist.wrist_flex_key,
			arm_config.wrist.wrist_roll_key,
			name=arm_config.name,
		)
		self.controller = ArmTeleopController(arm_config, kinematics)
		self.hold_action: dict[str, float] | None = None
		self.has_recalibration_request = False

	@property
	def action_keys(self) -> list[str]:
		"""이 팔이 내보내는 액션 키 목록 (접두사 포함)."""
		joint_keys = list(self.arm_config.ik.arm_joint_keys)
		joint_keys.append(self.arm_config.gripper.key)
		return [f'{self.prefix}{key}' for key in joint_keys]


class BiVrLeader(Teleoperator):
	"""VR 컨트롤러(좌/우)로 팔로워암을 조종하는 텔레오퍼레이터.

	사용 순서:
		1. connect() - SteamVR 연결
		2. initialize_from_observation(robot.get_observation()) -
		   로봇 실제 관절각으로 궤적/유지 액션 초기화 (필수)
		3. 매 사이클 get_action() - 최초 유효 pose가 팔별 영점이 되고,
		   이후 VR 델타만큼 로봇이 움직인다

	Attributes:
		config: BiVrLeaderConfig.
		vr_system: SteamVR 래퍼 (외부에서 주입 시 세션 공유).
	"""

	config_class = BiVrLeaderConfig
	name = 'bi_vr_leader'

	def __init__(
		self,
		config: BiVrLeaderConfig,
		vr_system: 'VRSystem | None' = None,
	) -> None:
		super().__init__(config)
		self.config = config
		# vr_system을 주입받으면 세션 소유권은 외부(예: 카메라 헤드와
		# 공유)에 있으므로 connect/shutdown을 여기서 하지 않는다.
		self._has_own_vr_session = vr_system is None
		self.vr_system = vr_system if vr_system is not None else VRSystem()
		self._is_vr_connected = False
		self._is_initialized = False
		self._last_time = None

		is_dual = config.mode == 'dual'
		self.arms: list[_VrArmRuntime] = []
		for arm_config in get_arm_configs(config.mode):
			arm_config = self._apply_overrides(arm_config)
			prefix = (
				f'{arm_config.controller_role.value}_' if is_dual else ''
			)
			self.arms.append(_VrArmRuntime(arm_config, prefix))

	def _apply_overrides(self, arm_config: 'ArmConfig') -> 'ArmConfig':
		"""프리셋 ArmConfig에 CLI 설정값을 덮어써 새 복사본을 만든다."""
		ik_config = arm_config.ik
		if self.config.urdf_path is not None:
			ik_config = dataclasses.replace(
				ik_config, urdf_path=self.config.urdf_path
			)
		return dataclasses.replace(
			arm_config,
			ik=ik_config,
			scale_factor=self.config.scale_factor,
			gripper=GripperConfig(
				open_deg=self.config.gripper_open_pos,
				close_deg=self.config.gripper_close_pos,
			),
			camera=None,  # 카메라 헤드는 별도 컨트롤러가 담당
		)

	# ------------------------------------------------------------
	# Teleoperator 인터페이스
	# ------------------------------------------------------------
	@property
	def action_features(self) -> dict[str, type]:
		"""액션 키 -> 타입 매핑 (bi_so_leader와 동일한 키 구조)."""
		features: dict[str, type] = {}
		for runtime in self.arms:
			for key in runtime.action_keys:
				features[key] = float
		return features

	@property
	def feedback_features(self) -> dict[str, type]:
		"""피드백 미지원 (빈 딕셔너리)."""
		return {}

	@property
	def is_connected(self) -> bool:
		"""SteamVR 연결 여부."""
		return self._is_vr_connected

	def connect(self, calibrate: bool = True) -> None:
		"""SteamVR에 연결한다 (공유 세션이면 연결은 외부 책임).

		Args:
			calibrate: 미사용 (VR 영점은 최초 유효 pose에서 자동 설정).
		"""
		if self._has_own_vr_session:
			self.vr_system.connect()
		self._is_vr_connected = True
		print(
			'[bi_vr_leader] VR 팔 텔레옵: 컨트롤러를 편한 위치로 들면 '
			'최초 인식된 pose가 영점이 됩니다.'
		)

	@property
	def is_calibrated(self) -> bool:
		"""모든 팔의 VR 영점이 설정됐는지 여부."""
		return all(
			runtime.controller.is_calibrated() for runtime in self.arms
		)

	def calibrate(self) -> None:
		"""VR 영점 재설정 요청 - 다음 유효 pose가 새 영점이 된다."""
		for runtime in self.arms:
			runtime.has_recalibration_request = True

	def configure(self) -> None:
		"""추가 설정 없음."""

	def initialize_from_observation(
		self, observation: dict[str, Any],
	) -> None:
		"""로봇의 실제 관절각으로 궤적·유지 액션을 초기화한다 (필수).

		robot.connect() 후, 제어 루프 시작 전에 한 번 호출해야 한다.

		Args:
			observation: robot.get_observation() 결과 (접두사 포함 키).
		"""
		for runtime in self.arms:
			arm_observation = {}
			for key in runtime.arm_config.ik.arm_joint_keys:
				arm_observation[key] = float(
					observation[f'{runtime.prefix}{key}']
				)
			runtime.controller.initialize_from_observation(arm_observation)
			# pose 유실/캘리브레이션 중에는 현재 자세를 유지한다.
			hold = {
				f'{runtime.prefix}{key}': value
				for key, value in arm_observation.items()
			}
			gripper_key = (
				f'{runtime.prefix}{runtime.arm_config.gripper.key}'
			)
			hold[gripper_key] = float(
				observation.get(
					gripper_key, self.config.gripper_open_pos
				)
			)
			runtime.hold_action = hold
		self._is_initialized = True

	def get_action(self) -> dict[str, float]:
		"""VR 컨트롤러 pose로부터 이번 사이클의 관절 목표각을 계산한다.

		Returns:
			액션 키 -> 목표각 딕셔너리. pose가 아직 없으면 유지 액션.

		Raises:
			RuntimeError: initialize_from_observation()을 호출하지 않은
				경우.
		"""
		if not self._is_initialized:
			raise RuntimeError(
				'get_action() 전에 initialize_from_observation()을 '
				'호출해야 합니다 (로봇 관측값으로 영점 초기화).'
			)

		now = time.perf_counter()
		dt = (
			max(now - self._last_time, 1e-3)
			if self._last_time is not None else 1e-3
		)
		self._last_time = now

		poses = self.vr_system.get_all_poses()
		action: dict[str, float] = {}
		for runtime in self.arms:
			action.update(self._compute_arm_action(runtime, poses, now, dt))
		return action

	def _compute_arm_action(
		self,
		runtime: _VrArmRuntime,
		poses: object,
		now: float,
		dt: float,
	) -> dict[str, float]:
		"""팔 하나의 액션 계산 (pose 유실 시 유지 액션 반환)."""
		device_index = self.vr_system.get_controller_index(
			runtime.arm_config.controller_role
		)
		if device_index is None:
			return dict(runtime.hold_action)

		pose = poses[device_index]
		if not pose.bPoseIsValid:
			return dict(runtime.hold_action)

		pose_matrix = pose.mDeviceToAbsoluteTracking
		vr_pos = self.vr_system.extract_position(pose_matrix)
		vr_rot = self.vr_system.extract_rotation_matrix(pose_matrix)

		is_calibration_needed = (
			not runtime.controller.is_calibrated()
			or runtime.has_recalibration_request
		)
		if is_calibration_needed:
			runtime.controller.calibrate(vr_pos, vr_rot)
			runtime.has_recalibration_request = False
			return dict(runtime.hold_action)

		trigger = self.vr_system.get_trigger_value(device_index)
		command = runtime.controller.compute(vr_pos, vr_rot, trigger, now, dt)

		action = {
			f'{runtime.prefix}{key}': value
			for key, value in command.joint_deg.items()
		}
		gripper_key = f'{runtime.prefix}{runtime.arm_config.gripper.key}'
		action[gripper_key] = command.gripper_deg
		runtime.hold_action = dict(action)
		return action

	def send_feedback(self, feedback: dict[str, Any]) -> None:
		"""피드백 미지원 (무시)."""

	def disconnect(self) -> None:
		"""SteamVR 세션을 종료한다 (공유 세션이면 종료는 외부 책임)."""
		if self._has_own_vr_session and self._is_vr_connected:
			self.vr_system.shutdown()
		self._is_vr_connected = False
