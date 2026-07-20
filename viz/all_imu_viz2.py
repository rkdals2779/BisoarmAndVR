import openvr
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque


def extract_euler_from_matrix(mat):
	"""OpenVR 3x4 변환 행렬에서 오차에 안전한 atan2 방식으로 Roll, Pitch, Yaw(도) 추출"""
	pitch = math.degrees(math.atan2(-mat[1][2], math.sqrt(mat[0][2] ** 2 + mat[2][2] ** 2)))
	yaw = math.degrees(math.atan2(mat[0][2], mat[2][2]))
	roll = math.degrees(math.atan2(mat[1][0], mat[1][1]))
	return roll, pitch, yaw


def find_vr_device_indices(vr_system):
	"""현재 연결된 HMD 및 좌우 컨트롤러의 고유 인덱스를 자동 탐색"""
	devices = {'hmd': openvr.k_unTrackedDeviceIndex_Hmd, 'left_ctrl': None, 'right_ctrl': None}

	for i in range(openvr.k_unMaxTrackedDeviceCount):
		device_class = vr_system.getTrackedDeviceClass(i)
		if device_class == openvr.TrackedDeviceClass_Controller:
			role = vr_system.getControllerRoleForTrackedDeviceIndex(i)
			if role == openvr.TrackedControllerRole_LeftHand:
				devices['left_ctrl'] = i
			elif role == openvr.TrackedControllerRole_RightHand:
				devices['right_ctrl'] = i
	return devices


def update_quiver_axes(ax, mat, x, y, z, quivers_list, scale=0.25):
	"""해당 장치의 변환 행렬을 기반으로 3D 공간에 R(X), G(Y), B(Z) 화살표 축을 플로팅"""
	q_x = ax.quiver(x, z, y, mat[0][0], mat[2][0], mat[1][0], color='red', length=scale, normalize=True, lw=1.5)
	q_y = ax.quiver(x, z, y, mat[0][1], mat[2][1], mat[1][1], color='green', length=scale, normalize=True, lw=1.5)
	q_z = ax.quiver(x, z, y, mat[0][2], mat[2][2], mat[1][2], color='blue', length=scale, normalize=True, lw=1.5)
	quivers_list.extend([q_x, q_y, q_z])


# 1. SteamVR 백엔드 초기화
try:
	vr_system = openvr.init(openvr.VRApplication_Background)
except openvr.OpenVRError as e:
	print(f'SteamVR을 시작할 수 없습니다: {e}')
	exit()

# 2. 실시간 대시보드 그래픽스 설정
plt.ion()
fig = plt.figure(figsize=(15, 7))
fig.canvas.manager.set_window_title('VR Full System 6-DoF Multi-Axis Analyzer')

# [좌측] 3D 공간 궤적 및 로컬 축 시각화
ax3d = fig.add_subplot(121, projection='3d')
ax3d.set_title('3D Vector Axes (HMD & Controllers)')
ax3d.set_xlim([-1.5, 1.5])
ax3d.set_ylim([-1.5, 1.5])
ax3d.set_zlim([0.0, 2.0])
ax3d.set_xlabel('X (Width)')
ax3d.set_ylabel('Z (Depth)')
ax3d.set_zlabel('Y (Height)')

# [우측] Yaw 각도 변화 비교 차트
ax2d = fig.add_subplot(122)
ax2d.set_title('Real-time Yaw Angle Tracking')
ax2d.set_xlim([0, 100])
ax2d.set_ylim([-180, 180])
ax2d.set_xlabel('Frames')
ax2d.set_ylabel('Yaw Degree (°)')
ax2d.grid(True, linestyle='--', alpha=0.5)

# 중심점 마커 설정
scatter_hmd = ax3d.scatter([], [], [], c='black', s=60, label='HMD (Head)')
scatter_left = ax3d.scatter([], [], [], c='darkred', s=40, label='Left Controller')
scatter_right = ax3d.scatter([], [], [], c='darkblue', s=40, label='Right Controller')
ax3d.legend(loc='upper left')

buffer_size = 100
history_hmd_yaw = deque(maxlen=buffer_size)
history_left_yaw = deque(maxlen=buffer_size)
history_right_yaw = deque(maxlen=buffer_size)

line_hmd_yaw, = ax2d.plot([], 'g-', label='HMD Yaw', lw=1.5)
line_left_yaw, = ax2d.plot([], 'r--', label='Left Ctrl Yaw', lw=1.5)
line_right_yaw, = ax2d.plot([], 'b--', label='Right Ctrl Yaw', lw=1.5)
ax2d.legend(loc='upper right')

active_quivers = []

print('======================================================================================')
print('             6자유도 멀티플 축(Axes) 시각화 시스템 가동 중...                          ')
print('======================================================================================')
# 초기 3줄 빈 공간 확보 (멀티라인 출력을 위해)
print('\n\n')

count = 0
try:
	while plt.fignum_exists(fig.number):
		device_map = find_vr_device_indices(vr_system)
		poses = vr_system.getDeviceToAbsoluteTrackingPose(
			openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
		)

		for q in active_quivers:
			q.remove()
		active_quivers.clear()

		# --- 1. HMD 처리 ---
		hmd_idx = device_map['hmd']
		hmd_pose = poses[hmd_idx]
		hmd_x, hmd_y, hmd_z = 0.0, 0.0, 0.0
		hmd_roll, hmd_pitch, hmd_yaw = 0.0, 0.0, 0.0
		hmd_status = 'OFF'

		if hmd_pose.bPoseIsValid:
			mat = hmd_pose.mDeviceToAbsoluteTracking
			hmd_x, hmd_y, hmd_z = mat[0][3], mat[1][3], mat[2][3]
			hmd_roll, hmd_pitch, hmd_yaw = extract_euler_from_matrix(mat)
			scatter_hmd._offsets3d = ([hmd_x], [hmd_z], [hmd_y])
			update_quiver_axes(ax3d, mat, hmd_x, hmd_y, hmd_z, active_quivers, scale=0.3)
			hmd_status = 'ON '
		history_hmd_yaw.append(hmd_yaw)

		# --- 2. 왼쪽 컨트롤러 처리 ---
		left_idx = device_map['left_ctrl']
		left_x, left_y, left_z = 0.0, 0.0, 0.0
		left_roll, left_pitch, left_yaw = 0.0, 0.0, 0.0
		left_status = 'OFF'

		if left_idx is not None and poses[left_idx].bPoseIsValid:
			mat = poses[left_idx].mDeviceToAbsoluteTracking
			left_x, left_y, left_z = mat[0][3], mat[1][3], mat[2][3]
			left_roll, left_pitch, left_yaw = extract_euler_from_matrix(mat)
			scatter_left._offsets3d = ([left_x], [left_z], [left_y])
			update_quiver_axes(ax3d, mat, left_x, left_y, left_z, active_quivers, scale=0.2)
			left_status = 'ON '
		else:
			scatter_left._offsets3d = ([], [], [])
		history_left_yaw.append(left_yaw)

		# --- 3. 오른쪽 컨트롤러 처리 ---
		right_idx = device_map['right_ctrl']
		right_x, right_y, right_z = 0.0, 0.0, 0.0
		right_roll, right_pitch, right_yaw = 0.0, 0.0, 0.0
		right_status = 'OFF'

		if right_idx is not None and poses[right_idx].bPoseIsValid:
			mat = poses[right_idx].mDeviceToAbsoluteTracking
			right_x, right_y, right_z = mat[0][3], mat[1][3], mat[2][3]
			right_roll, right_pitch, right_yaw = extract_euler_from_matrix(mat)
			scatter_right._offsets3d = ([right_x], [right_z], [right_y])
			update_quiver_axes(ax3d, mat, right_x, right_y, right_z, active_quivers, scale=0.2)
			right_status = 'ON '
		else:
			scatter_right._offsets3d = ([], [], [])
		history_right_yaw.append(right_yaw)

		# --- 💥 [핵심 수정] 멀티라인 텔레메트리 출력 제어 ---
		count += 1

		# 첫 번째 프레임이 아닐 때만 커서를 위로 3줄 올림 (\033[3A)
		if count > 1:
			print('\033[3A', end='')

		# 각 줄의 끝에 \033[K 를 붙여 현재 줄의 남은 이전 잔상을 완전히 깨끗하게 지움
		print(
			f'\n[{count:04d}] HMD    ({hmd_status}) -> XYZ:[{hmd_x: .2f},{hmd_y: .2f},{hmd_z: .2f}] RPY:[{hmd_roll: .1f},{hmd_pitch: .1f},{hmd_yaw: .1f}]\033[K')
		print(
			f'       L_Ctrl ({left_status}) -> XYZ:[{left_x: .2f},{left_y: .2f},{left_z: .2f}] RPY:[{left_roll: .1f},{left_pitch: .1f},{left_yaw: .1f}]\033[K')
		print(
			f'       R_Ctrl ({right_status}) -> XYZ:[{right_x: .2f},{right_y: .2f},{right_z: .2f}] RPY:[{right_roll: .1f},{right_pitch: .1f},{right_yaw: .1f}]\033[K',
			end='', flush=True)

		current_len = len(history_hmd_yaw)
		line_hmd_yaw.set_data(np.arange(current_len), list(history_hmd_yaw))
		line_left_yaw.set_data(np.arange(current_len), list(history_left_yaw))
		line_right_yaw.set_data(np.arange(current_len), list(history_right_yaw))

		fig.canvas.draw_idle()
		fig.canvas.flush_events()
		time.sleep(0.03)

except KeyboardInterrupt:
	print('\n\n[시스템] 모니터링을 정상 종료합니다.')
finally:
	plt.close('all')
	openvr.shutdown()