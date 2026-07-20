"""모드별 카메라 헤드 컨트롤러와 팩토리.

컨트롤러는 모두 같은 인터페이스(start/update/shutdown)를 가지며, 메인
텔레옵 루프에서 매 사이클 update()가 호출된다. 버스 쓰기는 전부 메인
루프 스레드에서만 일어난다 (공유 버스 패킷 충돌 방지 - 별도 스레드는
목표값 변수만 갱신한다).

keyboard/vr 모드는 별도 의존성(pynput/openvr 등)이 필요하므로 각각
keyboard_controller.py / vr_controller.py 모듈로 분리하고, 팩토리에서
해당 모드가 선택됐을 때만 임포트한다.
"""

import abc
import logging

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots import Robot

from .config import CameraHeadConfig, CameraHeadMode
from .driver import CameraHeadDriver

logger = logging.getLogger(__name__)


class CameraHeadControllerBase(abc.ABC):
	"""카메라 헤드 컨트롤러 공통 인터페이스.

	Attributes:
		config: 카메라 헤드 설정.
		driver: 공유 버스 드라이버.
		pan_deg: 마지막으로 전송한 pan 각도 (deg).
		tilt_deg: 마지막으로 전송한 tilt 각도 (deg).
	"""

	def __init__(
		self, config: CameraHeadConfig, driver: CameraHeadDriver,
	) -> None:
		self.config = config
		self.driver = driver
		self.pan_deg = config.pan_home_deg
		self.tilt_deg = config.tilt_home_deg
		self._is_shutdown_done = False

	def start(self) -> None:
		"""제어 시작 전 1회 호출 (기본: 아무것도 안 함)."""

	@abc.abstractmethod
	def update(self, t: float, dt: float) -> None:
		"""메인 루프에서 매 사이클 호출.

		Args:
			t: 이번 사이클 시각 (초, time.perf_counter 기준).
			dt: 이전 사이클 이후 경과 시간 (초).
		"""

	def shutdown(self) -> None:
		"""모터 토크 해제 등 정리 (중복 호출 안전).

		반드시 로봇 disconnect()(공유 버스 포트 닫힘)보다 먼저 호출해야
		한다.
		"""
		if self._is_shutdown_done:
			return
		self._is_shutdown_done = True
		print('[camera_head] 카메라 헤드 모터 토크 해제 중...')
		self.driver.release_torque()

	def _send(self, pan_deg: float, tilt_deg: float) -> None:
		"""리밋 clamp 후 전송하고 마지막 전송값을 기록한다."""
		self.pan_deg = self.config.clamp_pan(pan_deg)
		self.tilt_deg = self.config.clamp_tilt(tilt_deg)
		self.driver.write_pan_tilt(self.pan_deg, self.tilt_deg)


class FixedCameraHeadController(CameraHeadControllerBase):
	"""시작 시 고정 각도로 이동 후 유지하는 컨트롤러."""

	def start(self) -> None:
		"""고정 목표 각도(미지정 시 홈 각도)로 1회 이동 명령을 보낸다."""
		pan_deg = (
			self.config.fixed_pan_deg
			if self.config.fixed_pan_deg is not None
			else self.config.pan_home_deg
		)
		tilt_deg = (
			self.config.fixed_tilt_deg
			if self.config.fixed_tilt_deg is not None
			else self.config.tilt_home_deg
		)
		self._send(pan_deg, tilt_deg)
		print(
			f'[camera_head] 고정 모드: '
			f'pan {self.pan_deg:.1f}deg / tilt {self.tilt_deg:.1f}deg'
		)

	def update(self, t: float, dt: float) -> None:
		"""고정 모드는 매 사이클 할 일이 없다."""


def resolve_shared_bus(robot: Robot, bus_arm: str) -> FeetechMotorsBus:
	"""로봇 객체에서 카메라 헤드가 공유할 Feetech 버스를 찾는다.

	Args:
		robot: lerobot 로봇 인스턴스 (bi_so_follower 또는 단일 SO 팔).
		bus_arm: 헤드 모터가 물려 있는 팔 ('left'/'right').

	Returns:
		해당 팔의 FeetechMotorsBus.

	Raises:
		ValueError: 로봇에서 공유할 버스를 찾지 못한 경우.
	"""
	arm = getattr(robot, f'{bus_arm}_arm', None)
	if arm is not None and hasattr(arm, 'bus'):
		return arm.bus
	if hasattr(robot, 'bus'):
		return robot.bus
	raise ValueError(
		f'로봇 {robot.name!r}에서 카메라 헤드가 공유할 버스를 찾지 '
		f'못했습니다 (bus_arm={bus_arm!r}). bi_so_follower 또는 '
		'so101_follower 계열 로봇에서만 카메라 헤드를 쓸 수 있습니다.'
	)


def make_camera_head_controller(
	config: CameraHeadConfig,
	robot: Robot,
	vr_system: object | None = None,
	driver: CameraHeadDriver | None = None,
) -> CameraHeadControllerBase | None:
	"""설정에 맞는 카메라 헤드 컨트롤러를 생성한다.

	keyboard/vr 모드의 추가 의존성은 해당 모드를 선택했을 때만
	임포트한다.

	Args:
		config: 카메라 헤드 설정.
		robot: 연결이 끝난 lerobot 로봇 인스턴스.
		vr_system: vr 모드에서 공유할 VRSystem (None이면 자체 세션).
		driver: 재사용할 드라이버 (None이면 robot에서 버스를 찾아 생성.
			record처럼 관측 주입용 드라이버가 이미 있으면 그걸 넘긴다).

	Returns:
		모드에 맞는 컨트롤러. mode가 none이면 None.

	Raises:
		RuntimeError: 선택한 모드의 의존성이 설치돼 있지 않은 경우.
	"""
	if config.mode == CameraHeadMode.NONE:
		return None

	if driver is None:
		bus = resolve_shared_bus(robot, config.bus_arm)
		driver = CameraHeadDriver(
			bus, config.pan_motor_id, config.tilt_motor_id
		)

	if config.mode == CameraHeadMode.FIXED:
		return FixedCameraHeadController(config, driver)
	if config.mode == CameraHeadMode.KEYBOARD:
		from .keyboard_controller import KeyboardCameraHeadController
		return KeyboardCameraHeadController(config, driver)
	if config.mode == CameraHeadMode.VR:
		from .vr_controller import VrCameraHeadController
		return VrCameraHeadController(config, driver, vr_system=vr_system)
	raise ValueError(f'알 수 없는 카메라 헤드 모드: {config.mode!r}')
