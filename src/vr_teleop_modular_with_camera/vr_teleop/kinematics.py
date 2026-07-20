"""
로봇 팔 기구학(IK) 모듈
=======================
ikpy 체인 로딩, 활성 관절 마스크 계산, 손목 관절 인덱스 확인, 관절 한계
clip 등 IK와 관련된 모든 것을 RobotKinematics 클래스 하나에 캡슐화합니다.

여러 팔(로봇)을 동시에 돌릴 때도 팔마다 독립된 RobotKinematics
인스턴스를 가지므로 서로 상태가 섞이지 않습니다 (양팔 모드에서 로봇
2대가 서로 다른 IK 체인/관절 한계를 가져도 문제없이 동작합니다).
"""

import numpy as np
from ikpy.chain import Chain

from .config import IKConfig


class RobotKinematics:
	def __init__(self, ik_config: IKConfig, wrist_flex_key: str, wrist_roll_key: str, name: str = "arm"):
		self.config = ik_config
		self.name = name
		self.arm_joint_keys = list(ik_config.arm_joint_keys)

		print(f"[{self.name}] IK 체인 로딩: {ik_config.urdf_path}")
		self.chain = Chain.from_urdf_file(ik_config.urdf_path)

		# ikpy가 자동 생성하는 active_links_mask는 일부 URDF에서는 신뢰할 수
		# 없습니다 (예: 'fixed' 타입 조인트인데도 mask가 True로 잡히는 경우).
		# 대신 각 링크의 실제 joint_type을 직접 확인해서 'fixed'가 아닌 것만
		# 진짜 활성 관절(회전 모터)로 판단합니다.
		joint_types = [getattr(link, "joint_type", "fixed") for link in self.chain.links]
		self.active_mask = np.array([jt not in ("fixed", None) for jt in joint_types])
		n_active = int(self.active_mask.sum())

		print(f"[{self.name}][진단] IK 체인 링크:", [link.name for link in self.chain.links])
		print(f"[{self.name}][진단] joint_type:", joint_types)
		print(f"[{self.name}][진단] 실제 활성 관절 마스크:", self.active_mask, f"(활성 관절 수: {n_active})")

		assert n_active == len(self.arm_joint_keys), (
			f"[{self.name}] 체인의 활성 관절 수({n_active})가 arm_joint_keys 길이"
			f"({len(self.arm_joint_keys)})와 다릅니다.\n"
			"위에 출력된 링크 목록을 보고 IKConfig.arm_joint_keys 순서/개수를 실제 URDF에 맞게 수정하세요."
		)

		self.active_indices = np.where(self.active_mask)[0]

		# --- 위치 전용 IK 마스크: wrist_flex/wrist_roll은 최적화 대상에서 제외 ---
		# 이 두 관절은 control.py에서 컨트롤러 pitch/roll로 "직접" 계산해서
		# seed 값으로 고정합니다. active_links_mask에서 False로 두면 ikpy는
		# 이 두 관절을 최적화하지 않고 seed에 넣어준 값을 그대로(순기구학
		# 계산에는 반영하면서) 유지합니다 -> 위치 IK는 3DOF만 풀게 되어
		# 손목 회전과 절대 간섭하지 않습니다.
		self.wrist_flex_arm_idx = self.arm_joint_keys.index(wrist_flex_key)
		self.wrist_roll_arm_idx = self.arm_joint_keys.index(wrist_roll_key)
		self.wrist_flex_full_idx = int(self.active_indices[self.wrist_flex_arm_idx])
		self.wrist_roll_full_idx = int(self.active_indices[self.wrist_roll_arm_idx])

		position_only_mask = self.active_mask.copy()
		position_only_mask[self.wrist_flex_full_idx] = False
		position_only_mask[self.wrist_roll_full_idx] = False
		n_position_active = int(position_only_mask.sum())

		print(
			f"[{self.name}][진단] 위치 전용 IK 활성 관절 수:", n_position_active,
			"(wrist_flex/wrist_roll 제외, 3이어야 정상)",
		)
		assert n_position_active == 3, (
			f"[{self.name}] 위치 전용 IK 활성 관절이 3개가 아닙니다. "
			"arm_joint_keys 순서 또는 wrist_flex_key/wrist_roll_key 설정을 확인하세요."
		)

		# 이후 위치 IK 호출에서는 항상 이 마스크를 사용합니다.
		self.chain.active_links_mask = position_only_mask

		# --- 각 링크(관절)의 실제 물리적 한계(rad)를 URDF에서 추출 ---
		# scipy의 least_squares는 initial guess(seed)가 bounds를 단 하나라도
		# 벗어나면 최적화를 시작도 하지 않고 즉시 실패합니다. 그 실패한 seed가
		# 다음 프레임에 그대로 재사용되면 영원히 같은 에러만 반복되므로, 매번
		# seed를 물리적 한계 안으로 clip해서 이 악순환을 끊습니다.
		link_bounds = [getattr(link, "bounds", (None, None)) for link in self.chain.links]
		self.lower_bounds_full = np.array([
			(b[0] if (b is not None and b[0] is not None) else -np.inf) for b in link_bounds
		])
		self.upper_bounds_full = np.array([
			(b[1] if (b is not None and b[1] is not None) else np.inf) for b in link_bounds
		])

		self.wrist_flex_bounds_deg = (
			float(np.degrees(self.lower_bounds_full[self.wrist_flex_full_idx])),
			float(np.degrees(self.upper_bounds_full[self.wrist_flex_full_idx])),
		)
		self.wrist_roll_bounds_deg = (
			float(np.degrees(self.lower_bounds_full[self.wrist_roll_full_idx])),
			float(np.degrees(self.upper_bounds_full[self.wrist_roll_full_idx])),
		)

	def clip_to_bounds(self, full_angles_rad):
		return np.clip(full_angles_rad, self.lower_bounds_full, self.upper_bounds_full)

	def zeros_full(self):
		return np.zeros(len(self.chain.links))

	def build_seed(self, prev_full_rad, wrist_flex_rad, wrist_roll_rad):
		"""이전 IK 해를 물리적 한계 안으로 clip하고, 손목 두 관절은 이번
        프레임의 직접 계산값으로 덮어써서 IK seed를 만듭니다."""
		seed = self.clip_to_bounds(prev_full_rad).copy()
		seed[self.wrist_flex_full_idx] = wrist_flex_rad
		seed[self.wrist_roll_full_idx] = wrist_roll_rad
		return seed

	def solve_position_ik(self, target_pos, seed):
		angles_full = self.chain.inverse_kinematics(
			target_position=target_pos,
			initial_position=seed,
		)
		return self.clip_to_bounds(angles_full)

	def full_to_arm_deg(self, full_angles_rad):
		return np.degrees(np.asarray(full_angles_rad)[self.active_mask])

	def print_joint_diagnostics(self, initial_arm_deg):
		print(f"[{self.name}][진단] 관절 한계(deg) vs 현재 각도:")
		for key, idx, cur_deg in zip(self.arm_joint_keys, self.active_indices, initial_arm_deg):
			lo = self.lower_bounds_full[idx]
			hi = self.upper_bounds_full[idx]
			lo_deg = np.degrees(lo) if np.isfinite(lo) else -np.inf
			hi_deg = np.degrees(hi) if np.isfinite(hi) else np.inf
			flag = " <-- 이미 한계 밖!" if not (lo_deg - 1e-6 <= cur_deg <= hi_deg + 1e-6) else ""
			print(f"    [{self.name}] {key:20s}: [{lo_deg:8.1f}, {hi_deg:8.1f}]   현재: {cur_deg:8.1f}{flag}")
