"""
텔레옵 애플리케이션 (오케스트레이션)
====================================
config에 정의된 팔(들)을 받아서

    VR 초기화 -> 팔별 로봇/IK 초기화 -> 영점 동기화 -> 실시간 제어 루프

순서로 실행합니다. arms 리스트에 팔이 1개면 단일팔 모드, 2개(왼쪽+오른쪽)면
두 컨트롤러가 각각 다른 로봇을 동시에 움직이는 양팔(bimanual) 모드로
"자동으로" 동작합니다. 팔 개수 외에는 로직 차이가 전혀 없습니다.
"""

import time
from dataclasses import dataclass

from .camera_head import CameraHeadController
from .config import ArmConfig, TeleopConfig
from .control import ArmTeleopController
from .kinematics import RobotKinematics
from .output import ConsoleStatusDisplay, RobotOutput
from .vr_interface import VRSystem, HMD_DEVICE_INDEX


@dataclass
class _ArmRuntime:
	"""팔 하나에 필요한 런타임 객체 묶음 (내부용)"""
	config: ArmConfig
	kinematics: RobotKinematics
	output: RobotOutput
	# setup() 도중에는 output.connect()가 끝난 직후(=토크가 이미 켜진 직후)
	# 바로 self.arms에 등록해서, 그 뒤 단계(controller 초기화, 카메라
	# writer 탐색 등)에서 예외가 나도 shutdown()이 이 팔을 찾아 토크를
	# 해제할 수 있게 합니다. 그래서 controller는 처음엔 None일 수 있습니다.
	controller: ArmTeleopController | None = None
	device_index: int | None = None


class TeleopApp:
	def __init__(self, teleop_config: TeleopConfig) -> None:
		if not teleop_config.arms:
			raise ValueError(
				'TeleopConfig.arms가 비어 있습니다. '
				'최소 1개의 ArmConfig가 필요합니다.'
			)
		self.config = teleop_config
		self.vr_system = VRSystem()
		self.arms: list[_ArmRuntime] = []
		self.display = ConsoleStatusDisplay()

		# 팔 개수(1~2)와 무관하게, camera가 설정된 ArmConfig가 있으면
		# (기본: 왼팔) 카메라 헤드도 함께 초기화합니다. 그 팔의 RobotOutput이
		# 이미 열어놓은 시리얼 연결을 공유해서 쓰므로 별도 포트는 열지 않습니다.
		self.camera_head: CameraHeadController | None = None

		# shutdown()이 여러 경로(run()의 finally, setup() 실패 시 정리,
		# main()의 최종 안전망 등)에서 중복 호출될 수 있으므로, 실제
		# 정리 작업은 딱 한 번만 실행되도록 막는 플래그입니다.
		self._is_shutdown_done = False

	# ------------------------------------------------------------
	def setup(self) -> 'TeleopApp':
		# 이 메서드 전체를 try/except로 감싸는 이유: 로봇 연결(토크 ON)은
		# 팔마다 순서대로 일어나는데, 예를 들어 왼팔이 먼저 연결된 뒤
		# 오른팔 연결이나 카메라 시리얼 공유 탐색, IK 초기화 등 "그 다음
		# 단계"에서 예외가 나거나 이 시점에 Ctrl+C가 눌리면, 이미 토크가
		# 걸린 왼팔이 disconnect() 한 번 못 받아본 채로 프로세스가 죽습니다.
		# 그러면 다음 실행 때 모터가 여전히 뻣뻣한 상태로 남아 에러가 납니다.
		# 그래서 여기서 예외를 잡아 "그때까지 연결된 만큼만" 정리(토크 해제)
		# 하고 나서 원래 예외를 그대로 다시 던집니다.
		try:
			self.vr_system.connect()

			for arm_config in self.config.arms:
				print(
					f'\n---- [{arm_config.name}] '
					f'({arm_config.controller_role.value} 컨트롤러) 초기화 ----'
				)
				kinematics = RobotKinematics(
					arm_config.ik,
					arm_config.wrist.wrist_flex_key,
					arm_config.wrist.wrist_roll_key,
					name=arm_config.name,
				)
				output = RobotOutput(arm_config).connect()

				# 연결 성공 = 토크 ON 시점. 이후 단계가 실패하더라도
				# shutdown()이 이 팔을 반드시 찾을 수 있도록 controller가
				# 준비되기 "전에" 먼저 등록합니다.
				runtime = _ArmRuntime(
					config=arm_config, kinematics=kinematics, output=output
				)
				self.arms.append(runtime)

				controller = ArmTeleopController(arm_config, kinematics)
				controller.initialize_from_observation(output.get_observation())
				runtime.controller = controller

				if arm_config.camera is not None:
					print(
						f'[{arm_config.name}] 카메라 헤드 초기화 중... '
						f'(공유 포트: {arm_config.robot_port})'
					)
					camera_writer = output.get_camera_writer()
					self.camera_head = CameraHeadController(
						arm_config.camera, camera_writer
					)

			return self
		except BaseException:
			# BaseException을 잡는 이유: Ctrl+C(KeyboardInterrupt)는
			# Exception이 아니라 BaseException을 상속하므로, 일반
			# except Exception으로는 setup() 도중의 Ctrl+C를 못 잡습니다.
			print('\n[시스템] 초기화 도중 중단/오류 발생 - 연결된 로봇의 토크를 해제합니다.')
			self.shutdown()
			raise

	# ------------------------------------------------------------
	def run(self) -> None:
		dt_nominal = self.config.dt_nominal
		last_loop_t = time.perf_counter()
		try:
			# 카운트다운 대기도 try 안에 둬서, 이 시점에 Ctrl+C가 눌려도
			# finally에서 로봇 연결 해제가 항상 실행되도록 합니다.
			self.display.print_calibration_prompt()

			while True:
				loop_start = time.perf_counter()
				dt = max(loop_start - last_loop_t, 1e-3)
				last_loop_t = loop_start

				poses = self.vr_system.get_all_poses()
				has_sent_any = False

				# 카메라 헤드: get_all_poses()가 이미 받아온 결과에서 HMD
				# 항목만 꺼내 쓰므로 별도의 openvr 호출이 필요 없습니다.
				if self.camera_head is not None:
					hmd_pose = poses[HMD_DEVICE_INDEX]
					if hmd_pose.bPoseIsValid:
						hmd_rot = self.vr_system.extract_rotation_matrix(
							hmd_pose.mDeviceToAbsoluteTracking
						)
						if not self.camera_head.is_calibrated():
							self.camera_head.calibrate(hmd_rot)
						else:
							pan_deg, tilt_deg = (
								self.camera_head.compute_and_send(
									hmd_rot, loop_start, dt
								)
							)
							self.display.set_camera(pan_deg, tilt_deg)
							has_sent_any = True

				for runtime in self.arms:
					runtime.device_index = self.vr_system.get_controller_index(
						runtime.config.controller_role
					)
					if runtime.device_index is None:
						continue

					pose = poses[runtime.device_index]
					if not pose.bPoseIsValid:
						continue

					pose_matrix = pose.mDeviceToAbsoluteTracking
					vr_pos = self.vr_system.extract_position(pose_matrix)
					vr_rot = self.vr_system.extract_rotation_matrix(pose_matrix)

					if not runtime.controller.is_calibrated():
						# 최초로 유효한 pose를 받은 프레임 -> 이번 pose를 영점으로 저장하고
						# 이번 프레임은 명령을 보내지 않습니다 (델타가 항상 0에서 시작).
						runtime.controller.calibrate(vr_pos, vr_rot)
						continue

					trigger = self.vr_system.get_trigger_value(
						runtime.device_index
					)
					command = runtime.controller.compute(
						vr_pos, vr_rot, trigger, loop_start, dt
					)
					runtime.output.send(command)
					self.display.set(runtime.config.name, command)
					has_sent_any = True

				if has_sent_any:
					self.display.flush()

				elapsed = time.perf_counter() - loop_start
				time.sleep(max(0.0, dt_nominal - elapsed))

		except KeyboardInterrupt:
			print('\n\n[시스템] 안전하게 종료합니다.')
		finally:
			self.shutdown()

	# ------------------------------------------------------------
	def shutdown(self) -> None:
		# run()의 finally, setup() 실패 시 정리, main()의 최종 안전망(atexit
		# 등) 등 여러 경로가 이 메서드를 부를 수 있습니다. 두 번째 이후
		# 호출은 아무 것도 하지 않고 바로 리턴해서 "이미 닫힌 연결을 또
		# disconnect()하려다 나는 경고"가 반복 출력되는 것을 막습니다.
		if self._is_shutdown_done:
			return
		self._is_shutdown_done = True

		# 카메라 토크 해제는 반드시 로봇 팔 disconnect()보다 먼저 실행합니다.
		# disconnect()가 이 카메라와 공유 중인 시리얼 포트를 닫아버리므로,
		# 그 전에 마지막 패킷(토크 해제)을 먼저 내보내야 합니다.
		if self.camera_head is not None:
			try:
				print('[camera] 카메라 모터 토크 해제 중...')
				self.camera_head.release_torque()
				time.sleep(0.02)
			except Exception as error:
				print(f'[경고] 카메라 헤드 토크 해제 중 오류: {error}')

		for runtime in self.arms:
			try:
				print(f'[{runtime.config.name}] 모터 토크 해제 중...')
				runtime.output.disconnect()
			except Exception as error:
				print(f'[경고] [{runtime.config.name}] 로봇 연결 해제 중 오류: {error}')

		try:
			self.vr_system.shutdown()
		except Exception as error:
			print(f'[경고] VR 시스템 종료 중 오류: {error}')
