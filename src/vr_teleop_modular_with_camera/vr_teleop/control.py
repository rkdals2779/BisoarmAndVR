"""
제어부: 팔 하나에 대한 텔레옵 계산 파이프라인
==============================================
VR 컨트롤러의 raw 위치/회전과 로봇 관측값을 입력으로 받아서

    VR 위치(XYZ) -> One Euro Filter -> IK(3DOF, 위치만)
    VR 회전 -> pitch/roll 추출 -> One Euro Filter -> 손목 관절 직접 매핑
    (위치 3DOF + 손목 2DOF) -> 임계감쇠 궤적 컨트롤러
    트리거 -> 그리퍼

순서로 계산해서 "이번 프레임에 로봇에 보낼 관절 목표각(deg)"을 만드는
순수 계산 클래스입니다. openvr나 로봇 SDK를 전혀 알지 못하므로(직접
의존하지 않으므로) 시뮬레이션이나 유닛 테스트에서도 그대로 재사용할 수
있습니다.

--------------------------------------------------------------------
설계 노트 (원본 스크립트에서 이어받은 핵심 결정):
위치 3DOF는 IK로 풀고, 손목 pitch/roll 2DOF는 직접 관절 매핑, yaw는
미사용합니다. 컨트롤러의 orientation 전체를 IK의 target_orientation에
넣으면 5DOF 체인에 여유자유도가 생겨서 프레임마다 해가 다른 쪽으로
튀며 매우 불안정하게 움직입니다. 대신 물리적으로 대응되는 관절에 1:1로
직접 매핑하는 훨씬 안정적이고 예측 가능한 방식을 사용합니다.

    pitch (컨트롤러를 위아래로 끄덕이는 회전) -> wrist_flex
    roll  (컨트롤러를 손목처럼 비트는 회전)    -> wrist_roll
    yaw   (컨트롤러를 좌우로 젓는 회전)        -> 사용하지 않음
                                               (shoulder_pan이 좌우 위치를 담당)
--------------------------------------------------------------------
"""

from dataclasses import dataclass

import numpy as np

from .config import ArmConfig
from .filters import OneEuroFilter
from .geometry import extract_pitch_roll
from .kinematics import RobotKinematics
from .trajectory import JointTrajectoryController


@dataclass
class ArmCommand:
	"""이번 프레임에 실제로 로봇에 보낼 값 + 상태 표시용 정보"""
	joint_deg: dict[str, float]  # {joint_key: deg, ...} (그리퍼 제외)
	gripper_deg: float
	target_pos: np.ndarray
	wrist_flex_deg: float
	wrist_roll_deg: float


class ArmTeleopController:
	"""VR 컨트롤러 한쪽 + 로봇 한 대에 대한 텔레옵 계산 상태 머신.
    양팔 모드에서는 왼쪽/오른쪽 각각 독립된 인스턴스를 만들어 쓰면 됩니다."""

	def __init__(self, arm_config: ArmConfig, kinematics: RobotKinematics) -> None:
		self.config = arm_config
		self.kin = kinematics

		self.pos_filter = OneEuroFilter(
			min_cutoff=arm_config.position_filter.min_cutoff,
			beta=arm_config.position_filter.beta,
			d_cutoff=arm_config.position_filter.d_cutoff,
		)
		self.rot_filter = OneEuroFilter(
			min_cutoff=arm_config.wrist.filter.min_cutoff,
			beta=arm_config.wrist.filter.beta,
			d_cutoff=arm_config.wrist.filter.d_cutoff,
		)
		self.trajectory = JointTrajectoryController(
			kp=arm_config.trajectory.kp,
			max_vel_deg_s=arm_config.trajectory.max_vel_deg_s,
			max_acc_deg_s2=arm_config.trajectory.max_acc_deg_s2,
		)

		self._vr_home_pos: np.ndarray | None = None
		self._vr_home_rot: np.ndarray | None = None
		self._wrist_flex_home_deg = 0.0
		self._wrist_roll_home_deg = 0.0
		self._prev_target_pos: np.ndarray | None = None
		self._prev_ik_solution_full: np.ndarray | None = None

	# ------------------------------------------------------------
	# 초기화 / 영점 조절
	# ------------------------------------------------------------
	def initialize_from_observation(self, obs: dict[str, float]) -> None:
		"""로봇의 실제 현재 관절각으로 궤적/손목 영점을 초기화합니다.
        (추측값이 아니라 관측값으로 시작해야 시작 순간 로봇이 튀지 않습니다.)
        VR 캘리브레이션(calibrate)과는 별개로, 로봇에 연결한 직후 한 번만
        호출하면 됩니다."""
		initial_arm_deg = np.array([obs[k] for k in self.config.ik.arm_joint_keys])
		self.trajectory.reset(initial_arm_deg)

		self._wrist_flex_home_deg = float(initial_arm_deg[self.kin.wrist_flex_arm_idx])
		self._wrist_roll_home_deg = float(initial_arm_deg[self.kin.wrist_roll_arm_idx])

		self.kin.print_joint_diagnostics(initial_arm_deg)

		full = self.kin.zeros_full()
		full[self.kin.active_mask] = np.radians(initial_arm_deg)
		self._prev_ik_solution_full = self.kin.clip_to_bounds(full)

	def is_calibrated(self) -> bool:
		"""VR 컨트롤러 영점(home pose)이 설정되었는지 여부."""
		return self._vr_home_pos is not None

	def calibrate(self, vr_pos: np.ndarray, vr_rot: np.ndarray) -> None:
		"""이번 컨트롤러 pose를 '영점'으로 저장합니다. 최초로 유효한 pose를
        받은 프레임에서 1회 호출하면 되고, 이 시점 이후의 컨트롤러 델타만큼만
        로봇이 움직입니다."""
		self._vr_home_pos = vr_pos.copy()
		self._vr_home_rot = vr_rot.copy()

	# ------------------------------------------------------------
	# 매 프레임 계산
	# ------------------------------------------------------------
	def compute(self, vr_pos: np.ndarray, vr_rot: np.ndarray, trigger: float, t: float, dt: float) -> ArmCommand:
		"""calibrate()가 이미 호출된 상태에서 매 프레임 호출합니다."""
		cfg = self.config

		# 1. VR delta 위치 -> 로봇 좌표계 목표 위치
		delta_vr_pos = vr_pos - self._vr_home_pos
		mapped = np.array([
			-delta_vr_pos[2],
			-delta_vr_pos[0],
			delta_vr_pos[1],
		]) * cfg.scale_factor * np.array(cfg.axis_signs)
		raw_target_pos = cfg.home_pos + mapped

		# 2. One Euro Filter (위치 - 적응형 스무딩)
		target_pos = self.pos_filter.filter(raw_target_pos, t)

		# 2b. VR delta 회전 -> pitch/roll (yaw는 버림) -> One Euro Filter
		#     IK를 거치지 않고 wrist_flex/wrist_roll에 직접 매핑합니다.
		rel_rot = self._vr_home_rot.T @ vr_rot
		pitch_raw, roll_raw = extract_pitch_roll(rel_rot)
		pitch_filt, roll_filt = self.rot_filter.filter(np.array([pitch_raw, roll_raw]), t)

		wrist = cfg.wrist
		wrist_flex_deg = self._wrist_flex_home_deg + wrist.pitch_sign * np.degrees(pitch_filt) * wrist.pitch_scale
		wrist_roll_deg = self._wrist_roll_home_deg + wrist.roll_sign * np.degrees(roll_filt) * wrist.roll_scale
		wrist_flex_deg = float(np.clip(wrist_flex_deg, *self.kin.wrist_flex_bounds_deg))
		wrist_roll_deg = float(np.clip(wrist_roll_deg, *self.kin.wrist_roll_bounds_deg))

		# 3. IK - 위치만(3DOF). 실제로 움직였을 때만 재계산, 항상 이전 해로 시딩.
		#    wrist_flex/wrist_roll은 seed에 "이번 프레임의 직접 계산값"을 넣어서
		#    순기구학에는 반영되지만(팔 길이/오프셋 효과), active_links_mask가
		#    False라서 최적화 대상은 아닙니다.
		need_ik = (
			self._prev_target_pos is None
			or np.linalg.norm(target_pos - self._prev_target_pos) > cfg.ik.skip_threshold_m
		)
		seed = self.kin.build_seed(
			self._prev_ik_solution_full,
			np.radians(wrist_flex_deg),
			np.radians(wrist_roll_deg),
		)

		if need_ik:
			try:
				joint_angles_full = self.kin.solve_position_ik(target_pos, seed)
			except Exception as e:
				print(f'\n[경고][{cfg.name}] IK 실패, 이전 관절각 유지: {e}')
				joint_angles_full = seed
		else:
			joint_angles_full = seed

		# wrist_flex/wrist_roll은 IK와 완전히 무관하게, 매 프레임 항상 방금
		# 계산한 값으로 덮어씁니다. (need_ik가 False라서 위치 IK를 건너뛴
		# 프레임에도 손목 회전만은 매번 새로 갱신됩니다.)
		joint_angles_full[self.kin.wrist_flex_full_idx] = np.radians(wrist_flex_deg)
		joint_angles_full[self.kin.wrist_roll_full_idx] = np.radians(wrist_roll_deg)

		self._prev_ik_solution_full = joint_angles_full
		self._prev_target_pos = target_pos

		arm_target_deg = self.kin.full_to_arm_deg(joint_angles_full)

		# 4. 임계감쇠 궤적 컨트롤러 -> 속도/가속도가 제한된 매끄러운 움직임
		#    (위치 3DOF + 손목 pitch/roll 2DOF 모두 여기를 통과하므로
		#     손목이 갑자기 튀지 않고 부드럽게 따라옵니다.)
		arm_commanded_deg = self.trajectory.update(arm_target_deg, dt)

		# 5. 그리퍼 - IK/회전 매핑과 무관하게 트리거로 직접 제어
		gripper_deg = cfg.gripper.open_deg + trigger * (cfg.gripper.close_deg - cfg.gripper.open_deg)

		joint_deg = {key: float(arm_commanded_deg[i]) for i, key in enumerate(cfg.ik.arm_joint_keys)}

		return ArmCommand(
			joint_deg=joint_deg,
			gripper_deg=float(gripper_deg),
			target_pos=target_pos,
			wrist_flex_deg=wrist_flex_deg,
			wrist_roll_deg=wrist_roll_deg,
		)
