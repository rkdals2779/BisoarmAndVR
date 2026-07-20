"""팔로워 팔의 lerobot Feetech 버스를 공유하는 카메라 헤드 드라이버.

카메라 헤드 모터(id 7/8)는 팔로워 팔(모터 id 1~6)과 물리적으로 같은
시리얼 버스에 데이지체인으로 연결돼 있다. 같은 포트를 두 번 열면 두
연결의 패킷이 충돌해 양쪽 다 통신 오류가 나므로, 이 드라이버는 절대
새 시리얼 포트를 열지 않고 lerobot의 SOFollower가 이미 열어놓은
FeetechMotorsBus 객체를 그대로 빌려서 쓴다.

헤드 모터는 버스의 motors 딕셔너리에 등록돼 있지 않으므로 이름 기반
public API(write/sync_write) 대신 raw motor id를 받는 저수준 메서드
(_write/_disable_torque)를 사용한다. 이 메서드들은 writeTxRx로 응답
패킷까지 읽어 소비하므로 수신 버퍼에 잔여 바이트가 남지 않는다.
"""

import logging
from typing import Final

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.motors_bus import get_address

logger = logging.getLogger(__name__)

MOTOR_MODEL: Final[str] = 'sts3215'
ENCODER_COUNTS_PER_REV: Final[int] = 4096
MAX_ENCODER_COUNT: Final[int] = 4095


class CameraHeadDriver:
	"""공유 버스 위에서 pan/tilt 모터를 저수준 제어하는 드라이버.

	Attributes:
		bus: 팔로워 팔이 이미 열어놓은 lerobot Feetech 버스.
		pan_motor_id: pan 모터 id.
		tilt_motor_id: tilt 모터 id.
	"""

	def __init__(
		self,
		bus: FeetechMotorsBus,
		pan_motor_id: int,
		tilt_motor_id: int,
	) -> None:
		self.bus = bus
		self.pan_motor_id = pan_motor_id
		self.tilt_motor_id = tilt_motor_id
		self._goal_position_addr, self._goal_position_length = get_address(
			bus.model_ctrl_table, MOTOR_MODEL, 'Goal_Position'
		)

	def write_pan_tilt(self, pan_deg: float, tilt_deg: float) -> None:
		"""pan/tilt 목표 각도를 두 모터에 전송한다.

		Args:
			pan_deg: pan 목표 각도 (0~360 deg).
			tilt_deg: tilt 목표 각도 (0~360 deg).
		"""
		self._write_position(self.pan_motor_id, pan_deg)
		self._write_position(self.tilt_motor_id, tilt_deg)

	def _write_position(self, motor_id: int, position_deg: float) -> None:
		# 0~360도 -> 0~4095 엔코더 카운트 변환
		counts = int(position_deg * ENCODER_COUNTS_PER_REV / 360.0)
		counts = max(0, min(MAX_ENCODER_COUNT, counts))
		# 일시적 통신 오류로 텔레옵 루프 전체가 죽지 않도록 raise 대신
		# 로그만 남기고 다음 사이클에서 재시도되게 둔다.
		self.bus._write(
			self._goal_position_addr,
			self._goal_position_length,
			motor_id,
			counts,
			raise_on_error=False,
		)

	def release_torque(self) -> None:
		"""pan/tilt 모터의 토크를 해제한다 (손으로 움직이는 Free 상태).

		종료 시 반드시 로봇 disconnect()(버스 포트 닫힘)보다 먼저
		호출해야 한다.
		"""
		for motor_id in (self.pan_motor_id, self.tilt_motor_id):
			try:
				self.bus._disable_torque(motor_id, MOTOR_MODEL)
			except Exception as error:
				logger.warning(
					'카메라 헤드 모터 id=%d 토크 해제 실패: %s',
					motor_id, error,
				)
