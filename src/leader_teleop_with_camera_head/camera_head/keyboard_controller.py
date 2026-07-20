"""키보드로 카메라 헤드를 조작하는 컨트롤러.

pynput 전역 리스너 스레드는 키 눌림 상태(방향 변수)만 갱신하고, 실제
버스 쓰기는 메인 텔레옵 루프의 update()에서만 일어난다 (공유 버스에
스레드 두 개가 동시에 쓰면 패킷이 충돌하므로).

키 배치 (--camera_head.keyboard_keys):
	arrows (기본) : 좌/우=pan, 상/하=tilt, Home=홈 복귀
	wasd          : a/d=pan, w/s=tilt, h=홈 복귀
	              (record 중에는 방향키가 에피소드 제어와 겹치므로
	               wasd를 사용한다)

이동 방향이 반대면 아래 PAN_KEY_SIGN / TILT_KEY_SIGN 부호를 뒤집는다.
"""

from typing import Final

from .config import CameraHeadConfig
from .controllers import CameraHeadControllerBase
from .driver import CameraHeadDriver

try:
	from pynput import keyboard
except ImportError as error:
	raise RuntimeError(
		'keyboard 모드에는 pynput이 필요합니다. lerobot 환경에 '
		'`pip install pynput` 후 다시 실행하세요.'
	) from error

# 키 방향 부호. 실기에서 반대로 움직이면 부호를 뒤집는다.
PAN_KEY_SIGN: Final[float] = -1.0  # 실기 확인: -1.0이 왼쪽 화살표=왼쪽으로
TILT_KEY_SIGN: Final[float] = -1.0  # 실기 확인: -1.0이 위 화살표=위로

# 전송 생략 임계값: 목표가 이보다 적게 변하면 버스에 쓰지 않는다 (deg).
SEND_THRESHOLD_DEG: Final[float] = 0.02

# 키 배치: 역할 -> 키 객체
KEY_LAYOUTS: Final[dict[str, dict[str, object]]] = {
	'arrows': {
		'pan_positive': keyboard.Key.left,
		'pan_negative': keyboard.Key.right,
		'tilt_positive': keyboard.Key.up,
		'tilt_negative': keyboard.Key.down,
		'home': keyboard.Key.home,
	},
	'wasd': {
		'pan_positive': keyboard.KeyCode.from_char('a'),
		'pan_negative': keyboard.KeyCode.from_char('d'),
		'tilt_positive': keyboard.KeyCode.from_char('w'),
		'tilt_negative': keyboard.KeyCode.from_char('s'),
		'home': keyboard.KeyCode.from_char('h'),
	},
}

KEY_LAYOUT_GUIDES: Final[dict[str, str]] = {
	'arrows': '방향키 좌우=pan, 상하=tilt, Home=홈 복귀',
	'wasd': 'a/d=pan, w/s=tilt, h=홈 복귀',
}


class KeyboardCameraHeadController(CameraHeadControllerBase):
	"""키 누름 상태에 따라 pan/tilt를 증분 이동시키는 컨트롤러."""

	def __init__(
		self, config: CameraHeadConfig, driver: CameraHeadDriver,
	) -> None:
		super().__init__(config, driver)
		self._key_map = KEY_LAYOUTS[config.keyboard_keys]
		self._pan_direction = 0.0
		self._tilt_direction = 0.0
		self._has_home_request = False
		self._target_pan_deg = config.pan_home_deg
		self._target_tilt_deg = config.tilt_home_deg
		self._listener: keyboard.Listener | None = None

	def start(self) -> None:
		"""홈 각도로 이동하고 키보드 리스너를 시작한다."""
		self._send(self._target_pan_deg, self._target_tilt_deg)
		self._listener = keyboard.Listener(
			on_press=self._on_press_callback,
			on_release=self._on_release_callback,
		)
		self._listener.start()
		guide = KEY_LAYOUT_GUIDES[self.config.keyboard_keys]
		print(f'[camera_head] 키보드 모드: {guide}')

	def update(self, t: float, dt: float) -> None:
		"""키 눌림 상태를 적분해 목표각을 갱신하고 필요 시 전송한다."""
		if self._has_home_request:
			self._has_home_request = False
			self._target_pan_deg = self.config.pan_home_deg
			self._target_tilt_deg = self.config.tilt_home_deg
			self._send(self._target_pan_deg, self._target_tilt_deg)
			return

		speed = self.config.keyboard_speed_deg_s
		new_pan_deg = self.config.clamp_pan(
			self._target_pan_deg
			+ PAN_KEY_SIGN * self._pan_direction * speed * dt
		)
		new_tilt_deg = self.config.clamp_tilt(
			self._target_tilt_deg
			+ TILT_KEY_SIGN * self._tilt_direction * speed * dt
		)

		has_moved = (
			abs(new_pan_deg - self._target_pan_deg) > SEND_THRESHOLD_DEG
			or abs(new_tilt_deg - self._target_tilt_deg) > SEND_THRESHOLD_DEG
		)
		self._target_pan_deg = new_pan_deg
		self._target_tilt_deg = new_tilt_deg
		if has_moved:
			self._send(self._target_pan_deg, self._target_tilt_deg)

	def shutdown(self) -> None:
		"""리스너를 멈추고 토크를 해제한다 (중복 호출 안전)."""
		if self._listener is not None:
			self._listener.stop()
			self._listener = None
		super().shutdown()

	def _on_press_callback(self, key: object) -> None:
		# pynput 리스너 스레드: 방향 변수만 갱신한다 (버스 접근 금지).
		if key == self._key_map['pan_positive']:
			self._pan_direction = 1.0
		elif key == self._key_map['pan_negative']:
			self._pan_direction = -1.0
		elif key == self._key_map['tilt_positive']:
			self._tilt_direction = 1.0
		elif key == self._key_map['tilt_negative']:
			self._tilt_direction = -1.0
		elif key == self._key_map['home']:
			self._has_home_request = True

	def _on_release_callback(self, key: object) -> None:
		pan_keys = (
			self._key_map['pan_positive'], self._key_map['pan_negative']
		)
		tilt_keys = (
			self._key_map['tilt_positive'], self._key_map['tilt_negative']
		)
		if key in pan_keys:
			self._pan_direction = 0.0
		elif key in tilt_keys:
			self._tilt_direction = 0.0
