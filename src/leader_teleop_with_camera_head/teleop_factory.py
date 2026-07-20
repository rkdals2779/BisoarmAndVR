"""팔 텔레오퍼레이터 팩토리와 카메라 헤드 통합 래퍼.

- make_arm_teleoperator(): --teleop.type이 bi_vr_leader면 자체 VR 팔
  텔레옵을, 그 외에는 lerobot 팩토리를 사용한다.
- TeleopWithCameraHead: lerobot의 record_loop/teleop_loop가 매 사이클
  호출하는 get_action() 안에서 카메라 헤드도 함께 갱신하고, 헤드
  명령값을 액션에 포함시키는 래퍼.
- RobotWithCameraHead: 관측에 헤드 실측 각도를 추가하고, 전송 직전
  액션에서 헤드 키를 제거하는 로봇 래퍼 (데이터셋 feature에 헤드
  상태가 포함되게 하는 장치).

두 래퍼 덕분에 upstream 루프/데이터셋 코드를 수정하지 않고도 카메라
헤드가 함께 제어·기록된다 (버스 쓰기는 여전히 메인 루프 스레드에서만
일어난다).
"""

import sys
import time
from pathlib import Path
from typing import Any, Final

from lerobot.robots import Robot
from lerobot.teleoperators import (
	TeleoperatorConfig,
	make_teleoperator_from_config,
)
from lerobot.teleoperators.teleoperator import Teleoperator

from camera_head.config import CameraHeadConfig
from camera_head.controllers import CameraHeadControllerBase
from camera_head.driver import CameraHeadDriver
from vr_arm.config import BiVrLeaderConfig

_VR_TELEOP_PARENT_DIR: Final[Path] = (
	Path(__file__).resolve().parent.parent / 'vr_teleop_modular_with_camera'
)

# 데이터셋 action/observation에 기록되는 카메라 헤드 키
CAMERA_HEAD_PAN_KEY: Final[str] = 'camera_head_pan.pos'
CAMERA_HEAD_TILT_KEY: Final[str] = 'camera_head_tilt.pos'


def make_vr_system() -> object:
	"""vr_teleop의 VRSystem 인스턴스를 생성한다 (연결은 호출부 책임).

	VR 팔 텔레옵과 VR 카메라 헤드가 하나의 SteamVR 세션을 공유할 때
	사용한다 (openvr.init은 프로세스당 한 번만 해야 하므로).

	Returns:
		연결 전 상태의 VRSystem.

	Raises:
		RuntimeError: openvr/scipy 등 VR 의존성이 없는 경우.
	"""
	if str(_VR_TELEOP_PARENT_DIR) not in sys.path:
		sys.path.insert(0, str(_VR_TELEOP_PARENT_DIR))
	try:
		from vr_teleop.vr_interface import VRSystem
	except ImportError as error:
		raise RuntimeError(
			f'VR 모드에 필요한 의존성이 없습니다: {error}'
		) from error
	return VRSystem()


def make_arm_teleoperator(
	teleop_config: TeleoperatorConfig,
	vr_system: object | None = None,
) -> Teleoperator:
	"""팔 조종 소스 설정에 맞는 텔레오퍼레이터를 생성한다.

	Args:
		teleop_config: --teleop.* 설정 (bi_so_leader / bi_vr_leader 등).
		vr_system: bi_vr_leader에서 공유할 VRSystem (None이면 자체 세션).

	Returns:
		리더암(lerobot 팩토리) 또는 VR 컨트롤러(BiVrLeader)
		텔레오퍼레이터.
	"""
	if isinstance(teleop_config, BiVrLeaderConfig):
		from vr_arm.teleop import BiVrLeader
		return BiVrLeader(teleop_config, vr_system=vr_system)
	return make_teleoperator_from_config(teleop_config)


class TeleopWithCameraHead(Teleoperator):
	"""get_action() 호출 시 카메라 헤드도 함께 갱신하는 래퍼.

	lerobot의 record_loop는 매 사이클 teleop.get_action()을 호출하므로,
	이 래퍼를 넘기면 upstream 루프를 수정하지 않고도 카메라 헤드가 같은
	주기로 갱신된다. 그 외 모든 동작은 내부 텔레오퍼레이터에 위임한다.

	Attributes:
		inner: 실제 팔 텔레오퍼레이터.
		camera_head: 카메라 헤드 컨트롤러.
	"""

	config_class = TeleoperatorConfig  # 직접 생성 전용 (CLI 등록 안 함)
	name = 'teleop_with_camera_head'

	def __init__(
		self,
		inner: Teleoperator,
		camera_head: CameraHeadControllerBase,
	) -> None:
		# Teleoperator.__init__은 캘리브레이션 파일 경로 생성 등 config
		# 기반 부수효과가 있어 호출하지 않는다 - 래퍼는 자체 config가
		# 없고 캘리브레이션도 내부 텔레옵이 담당한다.
		self.inner = inner
		self.camera_head = camera_head
		self.id = inner.id
		self._last_time: float | None = None

	@property
	def action_features(self) -> dict:
		"""내부 텔레옵 액션 feature + 카메라 헤드 키."""
		return {
			**self.inner.action_features,
			CAMERA_HEAD_PAN_KEY: float,
			CAMERA_HEAD_TILT_KEY: float,
		}

	@property
	def feedback_features(self) -> dict:
		"""내부 텔레옵의 피드백 feature를 그대로 노출한다."""
		return self.inner.feedback_features

	@property
	def is_connected(self) -> bool:
		"""내부 텔레옵 연결 여부."""
		return self.inner.is_connected

	def connect(self, calibrate: bool = True) -> None:
		"""내부 텔레옵을 연결한다."""
		self.inner.connect(calibrate)

	@property
	def is_calibrated(self) -> bool:
		"""내부 텔레옵 캘리브레이션 여부."""
		return self.inner.is_calibrated

	def calibrate(self) -> None:
		"""내부 텔레옵 캘리브레이션을 위임한다."""
		self.inner.calibrate()

	def configure(self) -> None:
		"""내부 텔레옵 설정을 위임한다."""
		self.inner.configure()

	def get_action(self) -> dict[str, Any]:
		"""카메라 헤드를 갱신하고 헤드 명령값을 포함한 액션을 반환한다."""
		now = time.perf_counter()
		dt = (
			max(now - self._last_time, 1e-3)
			if self._last_time is not None else 1e-3
		)
		self._last_time = now
		self.camera_head.update(now, dt)
		action = dict(self.inner.get_action())
		action[CAMERA_HEAD_PAN_KEY] = self.camera_head.pan_deg
		action[CAMERA_HEAD_TILT_KEY] = self.camera_head.tilt_deg
		return action

	def send_feedback(self, feedback: dict[str, Any]) -> None:
		"""내부 텔레옵에 피드백을 위임한다."""
		self.inner.send_feedback(feedback)

	def disconnect(self) -> None:
		"""카메라 헤드 정리(중복 호출 안전) 후 내부 텔레옵을 끊는다."""
		self.camera_head.shutdown()
		self.inner.disconnect()


class RobotWithCameraHead(Robot):
	"""관측/액션 feature에 카메라 헤드 상태를 추가하는 로봇 래퍼.

	- observation_features / get_observation: 헤드 실측 각도
	  (camera_head_pan.pos / camera_head_tilt.pos)를 추가한다.
	  실측 읽기에 실패하면 마지막 전송값 -> 홈 각도 순으로 fallback.
	- action_features: 헤드 키를 추가한다 (데이터셋 액션 feature는
	  robot.action_features에서 만들어지므로).
	- send_action: 실제 로봇으로 보내기 전에 헤드 키를 제거한다
	  (헤드 구동은 카메라 헤드 컨트롤러가 담당).

	카메라 헤드 mode가 none이면 이 래퍼를 쓰지 않아야 한다 - 그래야
	데이터셋에 헤드 feature가 생기지 않는다.

	Attributes:
		inner: 실제 lerobot 로봇.
		driver: 카메라 헤드 공유 버스 드라이버.
	"""

	config_class = None  # 직접 생성 전용 (CLI 등록 안 함)
	name = 'robot_with_camera_head'

	def __init__(
		self,
		inner: Robot,
		driver: CameraHeadDriver,
		head_config: CameraHeadConfig,
	) -> None:
		# Robot.__init__은 캘리브레이션 파일 경로 생성 등 config 기반
		# 부수효과가 있어 호출하지 않는다 - 래퍼는 자체 config가 없다.
		self.inner = inner
		self.driver = driver
		self.head_config = head_config
		# 데이터셋 robot_type 등에는 실제 로봇 이름을 그대로 쓴다.
		self.name = inner.name

	@property
	def cameras(self) -> dict:
		"""내부 로봇의 카메라 딕셔너리."""
		return getattr(self.inner, 'cameras', {})

	@property
	def observation_features(self) -> dict:
		"""내부 로봇 관측 feature + 카메라 헤드 키."""
		return {
			**self.inner.observation_features,
			CAMERA_HEAD_PAN_KEY: float,
			CAMERA_HEAD_TILT_KEY: float,
		}

	@property
	def action_features(self) -> dict:
		"""내부 로봇 액션 feature + 카메라 헤드 키."""
		return {
			**self.inner.action_features,
			CAMERA_HEAD_PAN_KEY: float,
			CAMERA_HEAD_TILT_KEY: float,
		}

	@property
	def is_connected(self) -> bool:
		"""내부 로봇 연결 여부."""
		return self.inner.is_connected

	def connect(self, calibrate: bool = True) -> None:
		"""내부 로봇을 연결한다."""
		self.inner.connect(calibrate)

	@property
	def is_calibrated(self) -> bool:
		"""내부 로봇 캘리브레이션 여부."""
		return self.inner.is_calibrated

	def calibrate(self) -> None:
		"""내부 로봇 캘리브레이션을 위임한다."""
		self.inner.calibrate()

	def configure(self) -> None:
		"""내부 로봇 설정을 위임한다."""
		self.inner.configure()

	def get_observation(self) -> dict[str, Any]:
		"""내부 로봇 관측에 카메라 헤드 실측 각도를 추가해 반환한다."""
		observation = dict(self.inner.get_observation())
		measured = self.driver.read_pan_tilt()
		if measured is not None:
			pan_deg, tilt_deg = measured
		else:
			pan_deg = (
				self.driver.last_pan_deg
				if self.driver.last_pan_deg is not None
				else self.head_config.pan_home_deg
			)
			tilt_deg = (
				self.driver.last_tilt_deg
				if self.driver.last_tilt_deg is not None
				else self.head_config.tilt_home_deg
			)
		observation[CAMERA_HEAD_PAN_KEY] = pan_deg
		observation[CAMERA_HEAD_TILT_KEY] = tilt_deg
		return observation

	def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
		"""헤드 키를 제거한 액션을 내부 로봇에 보낸다.

		헤드 구동은 카메라 헤드 컨트롤러가 같은 루프에서 담당하므로,
		여기서는 팔 액션만 전달하고 헤드 값은 반환에만 되돌려준다.
		"""
		arm_action = {
			key: value for key, value in action.items()
			if key not in (CAMERA_HEAD_PAN_KEY, CAMERA_HEAD_TILT_KEY)
		}
		sent_action = dict(self.inner.send_action(arm_action))
		for key in (CAMERA_HEAD_PAN_KEY, CAMERA_HEAD_TILT_KEY):
			if key in action:
				sent_action[key] = action[key]
		return sent_action

	def disconnect(self) -> None:
		"""내부 로봇 연결을 해제한다."""
		self.inner.disconnect()
