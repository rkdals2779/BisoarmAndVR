"""
VR 입력부
=========
OpenVR SDK를 감싸서 "컨트롤러가 지금 어디 있고 트리거를 얼마나
당겼는지"만 answer하는 얇은 래퍼입니다. IK/필터 등 제어 로직은 전혀
모르고, 좌/우 컨트롤러를 role로 조회할 수 있어서 양팔(bimanual) 모드에서
VRSystem 인스턴스 하나를 여러 팔이 공유합니다.
"""

from collections.abc import Sequence
from typing import Final

import openvr

from .config import ControllerRole
from .geometry import extract_position, extract_rotation_matrix, extract_pitch_roll, extract_yaw_pitch

# 다른 모듈에서 vr_interface를 통해서도 바로 쓸 수 있도록 재노출
__all__ = [
	'VRSystem', 'HMD_DEVICE_INDEX',
	'extract_position', 'extract_rotation_matrix', 'extract_pitch_roll', 'extract_yaw_pitch',
]

_ROLE_MAP: Final[dict[ControllerRole, int]] = {
	ControllerRole.LEFT: openvr.TrackedControllerRole_LeftHand,
	ControllerRole.RIGHT: openvr.TrackedControllerRole_RightHand,
}

# HMD는 항상 이 인덱스 하나로 고정되어 있습니다 (openvr 상수 재노출).
# get_all_poses()가 한 프레임에 컨트롤러+HMD를 전부 가져오므로, 카메라
# 헤드 추종용으로 별도의 openvr 호출을 추가할 필요가 없습니다.
HMD_DEVICE_INDEX: Final[int] = openvr.k_unTrackedDeviceIndex_Hmd


class VRSystem:
	"""SteamVR 세션 하나를 감싸는 얇은 래퍼.
    양팔 모드에서는 이 인스턴스 하나를 왼쪽/오른쪽 두 ArmTeleopController가
    함께 조회합니다 (컨트롤러마다 새로 연결할 필요 없음)."""

	def __init__(self) -> None:
		self._vr: openvr.IVRSystem | None = None

	def connect(self) -> 'VRSystem':
		print('[VR] SteamVR 초기화 중...')
		self._vr = openvr.init(openvr.VRApplication_Background)
		return self

	def shutdown(self) -> None:
		if self._vr is not None:
			openvr.shutdown()
			self._vr = None

	def get_controller_index(self, role: ControllerRole) -> int | None:
		target_role = _ROLE_MAP[role]
		for i in range(openvr.k_unMaxTrackedDeviceCount):
			if self._vr.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_Controller:
				if self._vr.getControllerRoleForTrackedDeviceIndex(i) == target_role:
					return i
		return None

	def get_all_poses(self) -> Sequence[openvr.TrackedDevicePose_t]:
		"""디바이스 전체의 pose를 한 번에 가져옵니다. 팔이 여러 개(양팔
        모드)여도 매 프레임 이 함수는 한 번만 호출하고 결과를 공유하세요."""
		return self._vr.getDeviceToAbsoluteTrackingPose(
			openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
		)

	def get_trigger_value(self, controller_index: int) -> float:
		"""0.0(뗌) ~ 1.0(완전히 당김). pyopenvr 버전에 따라 axis 인덱스가
        다를 수 있으니 실제 컨트롤러로 값이 잘 들어오는지 먼저 확인하세요."""
		is_valid, state = self._vr.getControllerState(controller_index)
		if not is_valid:
			return 0.0
		return float(state.rAxis[1].x)

	@staticmethod
	def get_hmd_pose(poses: Sequence[openvr.TrackedDevicePose_t]) -> openvr.TrackedDevicePose_t:
		"""get_all_poses()로 이미 받아온 poses 배열에서 HMD 항목만 꺼내는
        편의 함수 (카메라 헤드 추종용). 별도의 openvr 호출은 필요 없습니다."""
		return poses[HMD_DEVICE_INDEX]

	# geometry 모듈의 순수 함수를 그대로 노출 (호출부 편의용)
	extract_position = staticmethod(extract_position)
	extract_rotation_matrix = staticmethod(extract_rotation_matrix)
	extract_pitch_roll = staticmethod(extract_pitch_roll)
	extract_yaw_pitch = staticmethod(extract_yaw_pitch)
