"""
기하 변환 유틸리티 (하드웨어 의존성 없음)
==========================================
OpenVR pose 행렬에서 위치/회전을 뽑아내고, 회전행렬에서 pitch/roll을
분리하는 순수 수학 함수만 모아둡니다. openvr, 로봇 SDK 등 어떤 하드웨어
라이브러리에도 의존하지 않으므로(numpy/scipy만 사용) 시뮬레이션이나
유닛 테스트에서도 그대로 재사용할 수 있습니다.
"""

from collections.abc import Sequence

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation


def extract_position(pose_matrix: Sequence[Sequence[float]]) -> np.ndarray:
	"""pyopenvr의 HmdMatrix34_t(3x4)에서 위치(XYZ) 성분만 뽑아냅니다."""
	return np.array([pose_matrix[0][3], pose_matrix[1][3], pose_matrix[2][3]])


def extract_rotation_matrix(
	pose_matrix: Sequence[Sequence[float]],
) -> np.ndarray:
	"""pyopenvr의 HmdMatrix34_t(3x4)에서 회전 성분(3x3)만 뽑아냅니다."""
	return np.array([
		[pose_matrix[0][0], pose_matrix[0][1], pose_matrix[0][2]],
		[pose_matrix[1][0], pose_matrix[1][1], pose_matrix[1][2]],
		[pose_matrix[2][0], pose_matrix[2][1], pose_matrix[2][2]],
	])


def extract_pitch_roll(rel_rot_matrix: np.ndarray) -> tuple[float, float]:
	"""
    캘리브레이션 시점 대비 상대 회전행렬에서 pitch/roll만 추출합니다.

    OpenVR 컨트롤러 로컬 축 관례: X=오른쪽, Y=위, Z=컨트롤러 뒤쪽
    (즉 -Z가 실제로 컨트롤러가 가리키는 포인팅 방향).

    3축 중 2개(pitch, roll)만 쓰고 yaw는 버리는데, 순서를 'XYZ'로 잡아서
    "버리는 축(yaw=Y)"이 가운데 각도가 되게 했습니다. Tait-Bryan 분해는
    구조적으로 가운데 각도만 ±90도로 범위가 눌리고(짐벌락과 같은 이유),
    첫 번째/세 번째 각도는 ±180도 풀레인지가 나옵니다. yaw를 가운데로
    보내면 pitch(X, 첫 번째)와 roll(Z, 세 번째) 둘 다 풀레인지를 그대로 씁니다.
    """
	pitch, _yaw, roll = ScipyRotation.from_matrix(
		rel_rot_matrix
	).as_euler('XYZ', degrees=False)
	return float(pitch), float(roll)


def extract_yaw_pitch(rel_rot_matrix: np.ndarray) -> tuple[float, float]:
	"""
    캘리브레이션 시점 대비 HMD 상대 회전행렬에서 yaw/pitch만 추출합니다
    (카메라 pan/tilt 헤드 추종용).

    extract_pitch_roll과 반대로 여기서는 yaw(고개 좌우 젓기)가 필요하고
    roll(고개 옆으로 기울이기)은 버립니다. 그래서 순서를 'YXZ'로 잡아서
    "버리는 축(roll=Z)"이 가운데로 가게 하고, yaw(Y, 첫 번째)와
    pitch(X, 두 번째) 둘 다 풀레인지를 그대로 씁니다.
    """
	yaw, pitch, _roll = ScipyRotation.from_matrix(
		rel_rot_matrix
	).as_euler('YXZ', degrees=False)
	return float(yaw), float(pitch)
