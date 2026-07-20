"""카메라 헤드 텔레옵 설정.

카메라 헤드(pan/tilt 모터, 기본 id 7/8)는 팔로워 팔과 같은 시리얼 버스에
데이지체인으로 연결돼 있다고 가정한다. 조작 모드는 CameraHeadMode로
선택하며, CLI에서는 --camera_head.mode=keyboard 처럼 지정한다.
"""

from dataclasses import dataclass
from enum import Enum


class CameraHeadMode(str, Enum):
	"""카메라 헤드 조작 모드."""

	NONE = 'none'
	FIXED = 'fixed'
	KEYBOARD = 'keyboard'
	VR = 'vr'


@dataclass
class CameraHeadConfig:
	"""카메라 헤드 제어 설정.

	Attributes:
		mode: 조작 모드 (none/fixed/keyboard/vr).
		bus_arm: 헤드 모터가 데이지체인된 팔로워 팔 ('left'/'right').
		pan_motor_id: pan(좌우) 모터 id.
		tilt_motor_id: tilt(상하) 모터 id.
		pan_home_deg: pan 홈 각도 (deg).
		tilt_home_deg: tilt 홈 각도 (deg).
		pan_min_deg: pan 하한 (deg).
		pan_max_deg: pan 상한 (deg).
		tilt_min_deg: tilt 하한 (deg).
		tilt_max_deg: tilt 상한 (deg).
		fixed_pan_deg: fixed 모드 목표 pan (None이면 홈 각도).
		fixed_tilt_deg: fixed 모드 목표 tilt (None이면 홈 각도).
		keyboard_speed_deg_s: keyboard 모드 이동 속도 (deg/s).
		keyboard_keys: keyboard 모드 키 배치 ('arrows'/'wasd').
			record 중에는 방향키가 에피소드 제어(조기 종료/재녹화)에
			쓰이므로 wasd를 사용해야 한다.
		vr_yaw_scale: HMD yaw -> pan 배율.
		vr_pitch_scale: HMD pitch -> tilt 배율.
		vr_yaw_sign: pan 방향 부호 (반대로 돌면 -1.0으로 뒤집기).
		vr_pitch_sign: tilt 방향 부호.
		vr_filter_min_cutoff: VR 회전 One Euro Filter 기본 컷오프.
		vr_filter_beta: VR 회전 One Euro Filter 속도 계수.
		trajectory_kp: VR 모드 궤적 컨트롤러 반응성.
		max_vel_deg_s: VR 모드 관절 속도 제한 (deg/s).
		max_acc_deg_s2: VR 모드 관절 가속도 제한 (deg/s^2).
	"""

	mode: CameraHeadMode = CameraHeadMode.NONE
	bus_arm: str = 'left'

	pan_motor_id: int = 7
	tilt_motor_id: int = 8

	pan_home_deg: float = 180.0
	tilt_home_deg: float = 220.0

	pan_min_deg: float = 90.0
	pan_max_deg: float = 270.0
	tilt_min_deg: float = 110.0
	tilt_max_deg: float = 250.0

	fixed_pan_deg: float | None = None
	fixed_tilt_deg: float | None = None

	keyboard_speed_deg_s: float = 60.0
	keyboard_keys: str = 'arrows'

	vr_yaw_scale: float = 1.0
	vr_pitch_scale: float = -1.0
	vr_yaw_sign: float = -1.0
	vr_pitch_sign: float = 1.0
	vr_filter_min_cutoff: float = 1.0
	vr_filter_beta: float = 0.3

	trajectory_kp: float = 45.0
	max_vel_deg_s: float = 150.0
	max_acc_deg_s2: float = 600.0

	def __post_init__(self) -> None:
		"""설정값 유효성 검증.

		Raises:
			ValueError: bus_arm이나 각도 리밋이 잘못된 경우.
		"""
		if self.bus_arm not in ('left', 'right'):
			raise ValueError(
				f'bus_arm은 left/right 중 하나여야 합니다: {self.bus_arm!r}'
			)
		if self.keyboard_keys not in ('arrows', 'wasd'):
			raise ValueError(
				f'keyboard_keys는 arrows/wasd 중 하나여야 합니다: '
				f'{self.keyboard_keys!r}'
			)
		if not self.pan_min_deg < self.pan_max_deg:
			raise ValueError(
				f'pan 리밋이 잘못됐습니다: '
				f'[{self.pan_min_deg}, {self.pan_max_deg}]'
			)
		if not self.tilt_min_deg < self.tilt_max_deg:
			raise ValueError(
				f'tilt 리밋이 잘못됐습니다: '
				f'[{self.tilt_min_deg}, {self.tilt_max_deg}]'
			)

	def clamp_pan(self, pan_deg: float) -> float:
		"""pan 각도를 리밋 안으로 clamp한다."""
		return min(max(pan_deg, self.pan_min_deg), self.pan_max_deg)

	def clamp_tilt(self, tilt_deg: float) -> float:
		"""tilt 각도를 리밋 안으로 clamp한다."""
		return min(max(tilt_deg, self.tilt_min_deg), self.tilt_max_deg)
