r"""리더암 텔레오퍼레이션 + 카메라 헤드 동시 제어 진입점.

lerobot-teleoperate와 동일한 인자/동작(리더암 -> 팔로워암, 카메라 관측,
rerun 시각화)에 카메라 헤드(pan/tilt, 기본 모터 id 7/8) 제어를 더한
스크립트다. lerobot 소스는 수정하지 않고 설치본의 config/루프 구성
요소를 그대로 재사용한다 (lerobot 0.5.x / 0.6.x 호환).

사용 예 (기존 lerobot-teleoperate 인자 + --camera_head.* 만 추가):

	python main.py \
		--robot.type=bi_so_follower \
		--robot.id=bi_follower \
		--robot.left_arm_config.port=/dev/so101_follower_left \
		--robot.right_arm_config.port=/dev/so101_follower_right \
		--teleop.type=bi_so_leader \
		--teleop.id=bi_leader \
		--teleop.left_arm_config.port=/dev/so101_leader_left \
		--teleop.right_arm_config.port=/dev/so101_leader_right \
		--display_data=true \
		--camera_head.mode=keyboard

카메라(--robot.left_arm_config.cameras=... 등) 인자도
lerobot-teleoperate와 완전히 동일하게 동작한다 - 관측에 프레임이
포함되고 display_data=true면 rerun에 표시된다. 카메라를 포함한 전체
실행 커맨드는 README.md에 있다 (복사해서 그대로 실행 가능).

카메라 헤드 모드: none(기본) / fixed / keyboard(방향키) / vr(HMD 추종).
자세한 옵션과 의존성은 README.md 참고.
"""

import logging
import time
from dataclasses import asdict, dataclass, field
from pprint import pformat

from lerobot.configs import parser
from lerobot.processor import (
	RobotAction,
	RobotObservation,
	RobotProcessorPipeline,
	make_default_processors,
)
from lerobot.robots import Robot, make_robot_from_config
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig
from lerobot.teleoperators import Teleoperator, make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up

from camera_head import (
	CameraHeadConfig,
	CameraHeadControllerBase,
	make_camera_head_controller,
)

try:
	# lerobot >= 0.6: rerun/foxglove 백엔드 선택형 시각화 API
	from lerobot.utils.visualization_utils import (
		init_visualization,
		log_visualization_data,
		shutdown_visualization,
	)
except ImportError:
	# lerobot 0.5.x: rerun 전용 API를 0.6 시그니처로 감싼다
	import rerun

	from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

	def init_visualization(
		display_mode: str,
		session_name: str,
		ip: str | None = None,
		port: int | None = None,
	) -> None:
		"""lerobot 0.5.x용 init_visualization 호환 래퍼."""
		init_rerun(session_name=session_name, ip=ip, port=port)

	def log_visualization_data(
		display_mode: str,
		observation: RobotObservation | None = None,
		action: RobotAction | None = None,
		compress_images: bool = False,
	) -> None:
		"""lerobot 0.5.x용 log_visualization_data 호환 래퍼."""
		log_rerun_data(
			observation=observation,
			action=action,
			compress_images=compress_images,
		)

	def shutdown_visualization(display_mode: str) -> None:
		"""lerobot 0.5.x용 shutdown_visualization 호환 래퍼."""
		rerun.rerun_shutdown()


@dataclass
class TeleopWithCameraHeadConfig(TeleoperateConfig):
	"""lerobot TeleoperateConfig에 카메라 헤드 설정을 더한 config.

	Attributes:
		camera_head: 카메라 헤드(pan/tilt) 제어 설정.
	"""

	camera_head: CameraHeadConfig = field(default_factory=CameraHeadConfig)
	"""카메라 헤드(pan/tilt) 제어 설정 (--camera_head.mode=... 로 선택)."""


def teleop_with_camera_head_loop(
	teleop: Teleoperator,
	robot: Robot,
	camera_head: CameraHeadControllerBase | None,
	fps: int,
	teleop_action_processor: RobotProcessorPipeline[
		tuple[RobotAction, RobotObservation], RobotAction
	],
	robot_action_processor: RobotProcessorPipeline[
		tuple[RobotAction, RobotObservation], RobotAction
	],
	robot_observation_processor: RobotProcessorPipeline[
		RobotObservation, RobotObservation
	],
	display_data: bool = False,
	display_mode: str = 'rerun',
	duration: float | None = None,
	display_compressed_images: bool = False,
) -> None:
	"""lerobot teleop_loop와 동일한 제어 루프 + 카메라 헤드 갱신.

	리더암 액션을 읽어 팔로워암에 보내는 부분은 lerobot의 teleop_loop와
	같은 순서/주기로 동작하고, 매 사이클 끝에 카메라 헤드 update()가
	추가로 실행된다. 버스 쓰기는 모두 이 루프(단일 스레드)에서만
	일어난다.

	Args:
		teleop: 리더암 텔레오퍼레이터.
		robot: 팔로워 로봇.
		camera_head: 카메라 헤드 컨트롤러 (mode=none이면 None).
		fps: 루프 목표 주파수 (Hz).
		teleop_action_processor: 리더 액션 처리 파이프라인.
		robot_action_processor: 로봇 전송 전 액션 처리 파이프라인.
		robot_observation_processor: 관측 처리 파이프라인.
		display_data: True면 rerun/콘솔에 상태 표시.
		display_mode: 시각화 백엔드 ('rerun'/'foxglove').
		duration: 최대 실행 시간 (초, None이면 무한).
		display_compressed_images: 이미지 JPEG 압축 전송 여부.
	"""
	display_len = max(len(key) for key in robot.action_features)
	start = time.perf_counter()
	last_loop_start = start
	while True:
		loop_start = time.perf_counter()
		dt = max(loop_start - last_loop_start, 1e-3)
		last_loop_start = loop_start

		obs = robot.get_observation()

		raw_action = teleop.get_action()
		teleop_action = teleop_action_processor((raw_action, obs))
		robot_action_to_send = robot_action_processor((teleop_action, obs))
		_ = robot.send_action(robot_action_to_send)

		if camera_head is not None:
			camera_head.update(loop_start, dt)

		if display_data:
			obs_transition = robot_observation_processor(obs)
			log_visualization_data(
				display_mode,
				observation=obs_transition,
				action=teleop_action,
				compress_images=display_compressed_images,
			)

			extra_lines = 0 if camera_head is None else 1
			print('\n' + '-' * (display_len + 10))
			print(f'{"NAME":<{display_len}} | {"NORM":>7}')
			for motor, value in robot_action_to_send.items():
				print(f'{motor:<{display_len}} | {value:>7.2f}')
			if camera_head is not None:
				print(
					f'{"camera_head":<{display_len}} | '
					f'pan {camera_head.pan_deg:6.1f} '
					f'tilt {camera_head.tilt_deg:6.1f}'
				)
			move_cursor_up(len(robot_action_to_send) + 3 + extra_lines)

		dt_s = time.perf_counter() - loop_start
		precise_sleep(max(1 / fps - dt_s, 0.0))
		loop_s = time.perf_counter() - loop_start
		print(f'Teleop loop time: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)')
		move_cursor_up(1)

		if duration is not None and time.perf_counter() - start >= duration:
			return


@parser.wrap()
def teleoperate_with_camera_head(cfg: TeleopWithCameraHeadConfig) -> None:
	"""설정에 따라 텔레옵 + 카메라 헤드 제어를 실행한다.

	Args:
		cfg: CLI에서 파싱된 실행 설정.
	"""
	init_logging()
	logging.info(pformat(asdict(cfg)))
	# display_mode 필드는 lerobot 0.6부터 생겼다 (0.5.x는 rerun 고정).
	display_mode = getattr(cfg, 'display_mode', 'rerun')
	if cfg.display_data:
		init_visualization(
			display_mode,
			session_name='teleoperation',
			ip=cfg.display_ip,
			port=cfg.display_port,
		)
	display_compressed_images = (
		True
		if (
			cfg.display_data
			and cfg.display_ip is not None
			and cfg.display_port is not None
		)
		else cfg.display_compressed_images
	)

	teleop = make_teleoperator_from_config(cfg.teleop)
	robot = make_robot_from_config(cfg.robot)
	(
		teleop_action_processor,
		robot_action_processor,
		robot_observation_processor,
	) = make_default_processors()

	teleop.connect()
	robot.connect()

	camera_head = None
	try:
		# 카메라 헤드는 로봇 connect() 이후에 만들어야 한다 - 팔로워가
		# 열어놓은 시리얼 버스를 공유하기 때문이다.
		camera_head = make_camera_head_controller(cfg.camera_head, robot)
		if camera_head is not None:
			camera_head.start()

		teleop_with_camera_head_loop(
			teleop=teleop,
			robot=robot,
			camera_head=camera_head,
			fps=cfg.fps,
			display_data=cfg.display_data,
			display_mode=display_mode,
			duration=cfg.teleop_time_s,
			teleop_action_processor=teleop_action_processor,
			robot_action_processor=robot_action_processor,
			robot_observation_processor=robot_observation_processor,
			display_compressed_images=display_compressed_images,
		)
	except KeyboardInterrupt:
		pass
	finally:
		# 카메라 헤드 토크 해제는 반드시 robot.disconnect()(공유 버스
		# 포트 닫힘)보다 먼저 실행한다.
		if camera_head is not None:
			camera_head.shutdown()
		if cfg.display_data:
			shutdown_visualization(display_mode)
		teleop.disconnect()
		robot.disconnect()


def main() -> None:
	"""서드파티 플러그인 등록 후 텔레옵을 시작한다."""
	register_third_party_plugins()
	teleoperate_with_camera_head()


if __name__ == '__main__':
	main()
