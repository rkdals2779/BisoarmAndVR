"""VR HMD(헤드셋) 방향을 추종하는 카메라 헤드 컨트롤러.

SteamVR에서 HMD의 yaw/pitch를 읽어 카메라 헤드 pan/tilt로 매핑한다.
필터/궤적/기하 계산은 기존 vr_teleop 패키지(One Euro Filter, 임계감쇠
궤적 컨트롤러, extract_yaw_pitch)를 그대로 재사용한다 - 해당 패키지는
pip 설치본이 아니므로 레포 내 상대 경로를 sys.path에 추가해 임포트한다.

의존성: openvr, scipy, numpy (+ SteamVR/ALVR 실행 중이어야 함).
"""

import sys
from pathlib import Path
from typing import Final

from .config import CameraHeadConfig
from .controllers import CameraHeadControllerBase
from .driver import CameraHeadDriver

# vr_teleop 패키지 위치: {레포루트}/src/vr_teleop_modular_with_camera
_VR_TELEOP_PARENT_DIR: Final[Path] = (
	Path(__file__).resolve().parents[2] / 'vr_teleop_modular_with_camera'
)
if str(_VR_TELEOP_PARENT_DIR) not in sys.path:
	sys.path.insert(0, str(_VR_TELEOP_PARENT_DIR))

try:
	import numpy as np

	from vr_teleop.filters import OneEuroFilter
	from vr_teleop.geometry import extract_yaw_pitch
	from vr_teleop.trajectory import JointTrajectoryController
	from vr_teleop.vr_interface import HMD_DEVICE_INDEX, VRSystem
except ImportError as error:
	raise RuntimeError(
		'vr 모드에는 openvr/scipy/numpy가 필요합니다. lerobot 환경에 '
		'`pip install openvr scipy` 후 SteamVR(ALVR)을 실행한 상태에서 '
		f'다시 실행하세요. (원본 오류: {error})'
	) from error


class VrCameraHeadController(CameraHeadControllerBase):
	"""HMD yaw/pitch를 pan/tilt로 매핑해 추종하는 컨트롤러.

	최초로 유효한 HMD pose를 받은 프레임을 영점(홈)으로 잡고, 이후의
	상대 회전만큼 헤드를 움직인다.
	"""

	def __init__(
		self,
		config: CameraHeadConfig,
		driver: CameraHeadDriver,
		vr_system: 'VRSystem | None' = None,
	) -> None:
		super().__init__(config, driver)
		# vr_system을 주입받으면 세션 소유권은 외부(예: VR 팔 텔레옵과
		# 공유)에 있으므로 connect/shutdown을 여기서 하지 않는다.
		self._has_own_vr_session = vr_system is None
		self.vr_system = vr_system if vr_system is not None else VRSystem()
		self.rot_filter = OneEuroFilter(
			min_cutoff=config.vr_filter_min_cutoff,
			beta=config.vr_filter_beta,
		)
		self.trajectory = JointTrajectoryController(
			kp=config.trajectory_kp,
			max_vel_deg_s=config.max_vel_deg_s,
			max_acc_deg_s2=config.max_acc_deg_s2,
		)
		self.trajectory.reset(
			np.array([config.pan_home_deg, config.tilt_home_deg])
		)
		self._home_rot: np.ndarray | None = None

	def start(self) -> None:
		"""SteamVR에 연결한다 (영점은 최초 유효 pose에서 자동 설정)."""
		if self._has_own_vr_session:
			self.vr_system.connect()
		print(
			'[camera_head] VR 모드: HMD를 정면으로 향한 상태에서 '
			'최초 인식된 방향이 영점이 됩니다.'
		)

	def update(self, t: float, dt: float) -> None:
		"""HMD pose를 읽어 pan/tilt 목표를 계산·전송한다."""
		poses = self.vr_system.get_all_poses()
		hmd_pose = poses[HMD_DEVICE_INDEX]
		if not hmd_pose.bPoseIsValid:
			return

		hmd_rot = self.vr_system.extract_rotation_matrix(
			hmd_pose.mDeviceToAbsoluteTracking
		)

		if self._home_rot is None:
			# 최초 유효 프레임: 영점 저장 + 홈 각도로 이동
			self._home_rot = hmd_rot.copy()
			self._send(self.config.pan_home_deg, self.config.tilt_home_deg)
			return

		rel_rot = self._home_rot.T @ hmd_rot
		yaw_raw, pitch_raw = extract_yaw_pitch(rel_rot)
		yaw_filt, pitch_filt = self.rot_filter.filter(
			np.array([yaw_raw, pitch_raw]), t
		)

		config = self.config
		pan_target = config.clamp_pan(
			config.pan_home_deg
			+ config.vr_yaw_sign * np.degrees(yaw_filt) * config.vr_yaw_scale
		)
		tilt_target = config.clamp_tilt(
			config.tilt_home_deg
			+ config.vr_pitch_sign
			* np.degrees(pitch_filt) * config.vr_pitch_scale
		)

		smoothed = self.trajectory.update(
			np.array([pan_target, tilt_target]), dt
		)
		self._send(float(smoothed[0]), float(smoothed[1]))

	def shutdown(self) -> None:
		"""토크 해제 후 SteamVR 세션을 종료한다 (중복 호출 안전).

		공유 세션이면 세션 종료는 외부 책임이다.
		"""
		is_first_call = not self._is_shutdown_done
		super().shutdown()
		if is_first_call and self._has_own_vr_session:
			self.vr_system.shutdown()
