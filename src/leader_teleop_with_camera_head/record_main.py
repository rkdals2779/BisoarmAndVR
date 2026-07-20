r"""데이터 수집(lerobot-record) + 카메라 헤드 동시 제어 진입점.

lerobot-record와 동일한 인자/동작(에피소드 녹화, 데이터셋 저장, rerun
시각화, 방향키 에피소드 제어)에 카메라 헤드(pan/tilt, 기본 모터 id 7/8)
제어와 팔 조종 소스 선택을 더한 스크립트다. 에피소드 루프와
record_loop는 lerobot 설치본의 것을 수정 없이 그대로 사용한다.

팔 조종 소스 선택 (--teleop.type):
	bi_so_leader  - 리더암으로 팔로워암 조종 (lerobot 기본)
	bi_vr_leader  - VR 컨트롤러로 팔로워암 조종 (자체 구현)

카메라 헤드 모드 (--camera_head.mode):
	none(기본) / fixed / keyboard / vr
	주의: record는 방향키(->조기 종료, <-재녹화)와 ESC(중지)를 에피소드
	제어에 쓰므로, keyboard 모드는 자동으로 wasd 배치(a/d=pan,
	w/s=tilt, h=홈)로 전환된다.

카메라 헤드 데이터 기록:
	mode가 none이 아니면 데이터셋의 action(명령값)과 observation(실측값)
	에 camera_head_pan.pos / camera_head_tilt.pos 항목이 추가된다.
	mode=none이면 헤드 항목 없이 lerobot-record와 동일한 스키마가 된다.

사용 예:

	python record_main.py \
		--robot.type=bi_so_follower \
		--robot.id=bi_follower \
		--robot.left_arm_config.port=/dev/so101_follower_left \
		--robot.right_arm_config.port=/dev/so101_follower_right \
		--teleop.type=bi_so_leader \
		--teleop.id=bi_leader \
		--teleop.left_arm_config.port=/dev/so101_leader_left \
		--teleop.right_arm_config.port=/dev/so101_leader_right \
		--dataset.repo_id=roboseasy/pick_place \
		--dataset.single_task='물건을 집어 상자에 넣는다' \
		--dataset.num_episodes=10 \
		--dataset.push_to_hub=false \
		--display_data=true \
		--camera_head.mode=vr

카메라(--robot.*.cameras) 인자는 lerobot-record와 동일하게 동작한다.
전체 커맨드 예시는 README.md 참고.
"""

import logging
from dataclasses import asdict, dataclass, field
from pprint import pformat

from lerobot.common.control_utils import (
	sanity_check_dataset_robot_compatibility,
)
from lerobot.configs import parser
from lerobot.datasets import (
	LeRobotDataset,
	VideoEncodingManager,
	aggregate_pipeline_dataset_features,
	create_initial_features,
)
from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.scripts.lerobot_record import RecordConfig, record_loop
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import (
	init_visualization,
	shutdown_visualization,
)

from camera_head import CameraHeadConfig, CameraHeadMode
from camera_head.controllers import (
	make_camera_head_controller,
	resolve_shared_bus,
)
from camera_head.driver import CameraHeadDriver
from teleop_factory import (
	RobotWithCameraHead,
	TeleopWithCameraHead,
	make_arm_teleoperator,
	make_vr_system,
)
from vr_arm.config import BiVrLeaderConfig


@dataclass
class RecordWithCameraHeadConfig(RecordConfig):
	"""lerobot RecordConfig에 카메라 헤드 설정을 더한 config.

	Attributes:
		camera_head: 카메라 헤드(pan/tilt) 제어 설정.
	"""

	camera_head: CameraHeadConfig = field(default_factory=CameraHeadConfig)
	"""카메라 헤드(pan/tilt) 제어 설정 (--camera_head.mode=... 로 선택)."""


@parser.wrap()
def record_with_camera_head(
	cfg: RecordWithCameraHeadConfig,
) -> LeRobotDataset:
	"""카메라 헤드와 함께 데이터 수집을 실행한다.

	lerobot-record의 record() 흐름을 그대로 따르되, 카메라 헤드
	생성/시작/정리와 VR 팔 텔레옵 초기화를 끼워 넣는다. 에피소드
	루프(record_loop)는 lerobot 것을 수정 없이 사용한다.

	Args:
		cfg: CLI에서 파싱된 실행 설정.

	Returns:
		수집이 끝난 LeRobotDataset.
	"""
	init_logging()
	logging.info(pformat(asdict(cfg)))
	if cfg.display_data:
		init_visualization(
			cfg.display_mode,
			session_name='recording',
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

	# record는 방향키/ESC를 에피소드 제어에 쓰므로 카메라 헤드 키보드는
	# wasd 배치로 강제 전환한다 (키 충돌 방지).
	is_keyboard_head = cfg.camera_head.mode == CameraHeadMode.KEYBOARD
	if is_keyboard_head and cfg.camera_head.keyboard_keys == 'arrows':
		logging.warning(
			'record에서는 방향키가 에피소드 제어(조기 종료/재녹화)와 '
			'겹치므로 카메라 헤드 키보드를 wasd 배치로 전환합니다.'
		)
		cfg.camera_head.keyboard_keys = 'wasd'

	# VR 팔 + VR 카메라 헤드가 동시에 켜지면 SteamVR 세션을 하나만
	# 만들어 공유한다 (openvr.init은 프로세스당 1회).
	is_vr_arm = isinstance(cfg.teleop, BiVrLeaderConfig)
	is_vr_head = cfg.camera_head.mode == CameraHeadMode.VR
	shared_vr_system = (
		make_vr_system() if (is_vr_arm and is_vr_head) else None
	)

	robot = make_robot_from_config(cfg.robot)
	arm_teleop = make_arm_teleoperator(
		cfg.teleop, vr_system=shared_vr_system
	)

	# 카메라 헤드 사용 시 로봇을 래핑해 헤드 상태(pan/tilt)가 데이터셋
	# action/observation feature에 포함되게 한다. mode=none이면 래핑하지
	# 않으므로 헤드 데이터가 기록되지 않는다.
	camera_head_driver = None
	if cfg.camera_head.mode != CameraHeadMode.NONE:
		head_bus = resolve_shared_bus(robot, cfg.camera_head.bus_arm)
		camera_head_driver = CameraHeadDriver(
			head_bus,
			cfg.camera_head.pan_motor_id,
			cfg.camera_head.tilt_motor_id,
		)
		robot = RobotWithCameraHead(
			robot, camera_head_driver, cfg.camera_head
		)

	(
		teleop_action_processor,
		robot_action_processor,
		robot_observation_processor,
	) = make_default_processors()

	dataset_features = combine_feature_dicts(
		aggregate_pipeline_dataset_features(
			pipeline=teleop_action_processor,
			initial_features=create_initial_features(
				action=robot.action_features
			),
			use_videos=cfg.dataset.video,
		),
		aggregate_pipeline_dataset_features(
			pipeline=robot_observation_processor,
			initial_features=create_initial_features(
				observation=robot.observation_features
			),
			use_videos=cfg.dataset.video,
		),
	)

	dataset = None
	listener = None
	camera_head = None

	try:
		if cfg.resume:
			num_cameras = (
				len(robot.cameras) if hasattr(robot, 'cameras') else 0
			)
			dataset = LeRobotDataset.resume(
				cfg.dataset.repo_id,
				root=cfg.dataset.root,
				batch_encoding_size=cfg.dataset.video_encoding_batch_size,
				rgb_encoder=cfg.dataset.rgb_encoder,
				depth_encoder=cfg.dataset.depth_encoder,
				encoder_threads=cfg.dataset.encoder_threads,
				streaming_encoding=cfg.dataset.streaming_encoding,
				encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
				image_writer_processes=(
					cfg.dataset.num_image_writer_processes
					if num_cameras > 0 else 0
				),
				image_writer_threads=(
					cfg.dataset.num_image_writer_threads_per_camera
					* num_cameras
					if num_cameras > 0 else 0
				),
			)
			sanity_check_dataset_robot_compatibility(
				dataset, robot, cfg.dataset.fps, dataset_features
			)
		else:
			# eval_ 접두 데이터셋명은 정책 평가용으로 예약돼 있다.
			repo_name = cfg.dataset.repo_id.split('/', 1)[-1]
			if repo_name.startswith('eval_'):
				raise ValueError(
					"'eval_'로 시작하는 데이터셋 이름은 정책 평가용으로 "
					'예약돼 있습니다. 데이터 수집에는 다른 이름을 '
					'사용하세요.'
				)
			cfg.dataset.stamp_repo_id()
			dataset = LeRobotDataset.create(
				cfg.dataset.repo_id,
				cfg.dataset.fps,
				root=cfg.dataset.root,
				robot_type=robot.name,
				features=dataset_features,
				use_videos=cfg.dataset.video,
				image_writer_processes=(
					cfg.dataset.num_image_writer_processes
				),
				image_writer_threads=(
					cfg.dataset.num_image_writer_threads_per_camera
					* len(robot.cameras)
				),
				batch_encoding_size=cfg.dataset.video_encoding_batch_size,
				rgb_encoder=cfg.dataset.rgb_encoder,
				depth_encoder=cfg.dataset.depth_encoder,
				encoder_threads=cfg.dataset.encoder_threads,
				streaming_encoding=cfg.dataset.streaming_encoding,
				encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
			)

		if shared_vr_system is not None:
			shared_vr_system.connect()

		robot.connect()
		arm_teleop.connect()

		# 카메라 헤드는 로봇 connect() 이후에 시작해야 한다 - 팔로워가
		# 열어놓은 시리얼 버스를 공유하기 때문이다. 드라이버는 관측
		# 주입용으로 만든 것을 재사용한다.
		camera_head = make_camera_head_controller(
			cfg.camera_head,
			robot,
			vr_system=shared_vr_system,
			driver=camera_head_driver,
		)
		if camera_head is not None:
			camera_head.start()

		# VR 팔 텔레옵은 로봇의 실제 관절각으로 영점을 초기화해야 한다.
		if hasattr(arm_teleop, 'initialize_from_observation'):
			arm_teleop.initialize_from_observation(robot.get_observation())

		# record_loop가 매 사이클 get_action()을 부를 때 카메라 헤드도
		# 함께 갱신되도록 래퍼로 감싼다.
		teleop = (
			TeleopWithCameraHead(arm_teleop, camera_head)
			if camera_head is not None else arm_teleop
		)

		listener, events = init_keyboard_listener()

		with VideoEncodingManager(dataset):
			recorded_episodes = 0
			while (
				recorded_episodes < cfg.dataset.num_episodes
				and not events['stop_recording']
			):
				log_say(
					f'Recording episode {dataset.num_episodes}',
					cfg.play_sounds,
				)
				record_loop(
					robot=robot,
					events=events,
					fps=cfg.dataset.fps,
					teleop_action_processor=teleop_action_processor,
					robot_action_processor=robot_action_processor,
					robot_observation_processor=robot_observation_processor,
					teleop=teleop,
					dataset=dataset,
					control_time_s=cfg.dataset.episode_time_s,
					single_task=cfg.dataset.single_task,
					display_data=cfg.display_data,
					display_mode=cfg.display_mode,
					display_compressed_images=display_compressed_images,
				)

				# 환경 리셋 시간 (마지막 에피소드는 생략)
				if not events['stop_recording'] and (
					recorded_episodes < cfg.dataset.num_episodes - 1
					or events['rerecord_episode']
				):
					log_say('Reset the environment', cfg.play_sounds)

					record_loop(
						robot=robot,
						events=events,
						fps=cfg.dataset.fps,
						teleop_action_processor=teleop_action_processor,
						robot_action_processor=robot_action_processor,
						robot_observation_processor=(
							robot_observation_processor
						),
						teleop=teleop,
						control_time_s=cfg.dataset.reset_time_s,
						single_task=cfg.dataset.single_task,
						display_data=cfg.display_data,
						display_mode=cfg.display_mode,
					)

				if events['rerecord_episode']:
					log_say('Re-record episode', cfg.play_sounds)
					events['rerecord_episode'] = False
					events['exit_early'] = False
					dataset.clear_episode_buffer()
					continue

				dataset.save_episode()
				recorded_episodes += 1
	finally:
		log_say('Stop recording', cfg.play_sounds, blocking=True)

		if dataset:
			dataset.finalize()

		# 카메라 헤드 토크 해제는 반드시 robot.disconnect()(공유 버스
		# 포트 닫힘)보다 먼저 실행한다.
		if camera_head is not None:
			camera_head.shutdown()

		if robot.is_connected:
			robot.disconnect()
		if arm_teleop and arm_teleop.is_connected:
			arm_teleop.disconnect()
		if shared_vr_system is not None:
			shared_vr_system.shutdown()

		if listener is not None:
			listener.stop()

		if cfg.display_data:
			shutdown_visualization(cfg.display_mode)

		if cfg.dataset.push_to_hub:
			if dataset and dataset.num_episodes > 0:
				dataset.push_to_hub(
					tags=cfg.dataset.tags, private=cfg.dataset.private
				)
			else:
				logging.warning(
					'No episodes saved — skipping push to hub'
				)

		log_say('Exiting', cfg.play_sounds)
	return dataset


def main() -> None:
	"""서드파티 플러그인 등록 후 데이터 수집을 시작한다."""
	register_third_party_plugins()
	record_with_camera_head()


if __name__ == '__main__':
	main()
