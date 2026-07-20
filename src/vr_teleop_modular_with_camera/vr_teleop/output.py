"""
출력부: 로봇 액추에이터 명령 전송 + 콘솔 상태 표시
==================================================
제어부(control.py)가 계산한 ArmCommand를 실제 로봇 서보로 보내는
RobotOutput과, 사람이 보는 콘솔 상태줄을 찍는 ConsoleStatusDisplay를
담당합니다. 로봇 SDK(lerobot)를 직접 건드리는 코드는 이 파일에만 있으므로,
다른 로봇 드라이버로 교체하고 싶으면 이 파일만 고치면 됩니다.
"""

import time

from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower

from .camera_head import SharedFeetechWriter, find_shared_serial
from .config import ArmConfig
from .control import ArmCommand


class RobotOutput:
	"""SO-101 팔로워 로봇 한 대에 대한 연결/관측/명령 전송을 감싸는 출력부."""

	def __init__(self, arm_config: ArmConfig):
		self.config = arm_config
		self.robot = None

	def connect(self) -> "RobotOutput":
		print(f"[{self.config.name}] 로봇 초기화 중... (port={self.config.robot_port})")
		# disable_torque_on_disconnect=True: disconnect()가 호출될 때
		# lerobot이 모터 토크를 자동으로 꺼주도록 명시적으로 설정합니다.
		# (lerobot 기본값도 True이지만, 설치된 lerobot 버전이나 환경에 따라
		# 달라질 수 있으므로 여기서 명시해 둡니다.) 실제로 이게 동작하려면
		# disconnect()가 반드시 호출되어야 하고, 그건 app.py의 shutdown()이
		# Ctrl+C/에러/초기화 실패 등 모든 종료 경로에서 보장합니다.
		robot_cfg = SO101FollowerConfig(
			id=self.config.robot_id,
			port=self.config.robot_port,
			disable_torque_on_disconnect=True,
		)
		self.robot = SO101Follower(robot_cfg)
		self.robot.connect()
		return self

	def get_observation(self) -> dict:
		return self.robot.get_observation()

	def send(self, command: ArmCommand):
		action = dict(command.joint_deg)
		action[self.config.gripper.key] = command.gripper_deg
		self.robot.send_action(action)

	def get_camera_writer(self) -> "SharedFeetechWriter | None":
		"""
        이 팔의 config에 camera(CameraHeadConfig)가 설정되어 있으면, 이
        로봇이 이미 열어놓은 시리얼 연결을 그대로 공유하는 SharedFeetechWriter를
        만들어 반환합니다. 새 포트를 여는 대신 기존 연결 객체를 찾아
        재사용하므로, 로봇 팔과 카메라 모터가 물려 있는 같은 물리 버스에
        패킷이 충돌 없이 순서대로만 나갑니다.

        config.camera가 None이면 아무것도 하지 않고 None을 반환합니다.
        """
		if self.config.camera is None:
			return None
		if self.robot is None:
			raise RuntimeError(f"[{self.config.name}] connect()를 먼저 호출해야 합니다.")

		shared = find_shared_serial(self.robot)
		if shared is None:
			raise RuntimeError(
				f"[{self.config.name}] 카메라 헤드용 공유 시리얼 연결을 찾지 못했습니다. "
				"설치된 lerobot 버전의 내부 구조가 달라졌을 수 있습니다 "
				"(camera_head.find_shared_serial이 robot 객체 안에서 serial.Serial "
				"인스턴스를 찾지 못함). config.camera=None으로 두거나, lerobot의 "
				"SO101Follower/FeetechMotorsBus 내부에서 시리얼 연결이 어떤 속성에 "
				"저장되는지 확인 후 find_shared_serial의 탐색 로직을 맞춰주세요."
			)
		return SharedFeetechWriter(shared)

	def disconnect(self):
		if self.robot is not None:
			self.robot.disconnect()


class ConsoleStatusDisplay:
	"""팔이 1개(단일팔)든 2개(양팔)든 매 프레임 한 줄로 갱신되는 상태 표시.
    사용법: 루프마다 각 팔에 대해 set()을 호출한 뒤, 마지막에 flush() 1회."""

	def __init__(self):
		self._parts: dict = {}

	@staticmethod
	def print_calibration_prompt(seconds: int = 3):
		print("\n==================================================")
		print(" 컨트롤러를 편한 위치와 방향으로 들고 대기하세요. (영점 조절)")
		for i in range(seconds, 0, -1):
			print(f" {i}초 전...")
			time.sleep(1)
		print("\n[동기화 완료] 추종을 시작합니다! (종료: Ctrl+C)")
		print("==================================================\n")

	def set(self, name: str, command: ArmCommand):
		pos = command.target_pos
		self._parts[name] = (
			f"[{name}] XYZ:[{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}] "
			f"flex:{command.wrist_flex_deg:6.1f} roll:{command.wrist_roll_deg:6.1f} "
			f"grip:{command.gripper_deg:5.1f}"
		)

	def set_camera(self, pan_deg: float, tilt_deg: float):
		self._parts["camera"] = f"[camera] Pan:{pan_deg:6.1f} Tilt:{tilt_deg:6.1f}"

	def flush(self):
		if not self._parts:
			return
		line = "  |  ".join(self._parts[name] for name in self._parts)
		print(f"\r[추종 중] {line}   ", end="", flush=True)
