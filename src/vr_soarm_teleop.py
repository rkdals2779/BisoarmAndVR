import numpy as np
import time
import openvr
from ikpy.chain import Chain
from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower

# ============================================================
# 1. 환경 설정
# ============================================================
FOLLOWER_ARM_ID = "skm_right_follower"
FOLLOWER_ARM_PORT = "/dev/ttyACM1"
URDF_PATH = "/home/roboseasy/shin_ws/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"

# 로봇의 안전한 기준(Home) 좌표 (단위: m)
ROBOT_HOME_POS = np.array([0.2, 0.0, 0.1])
ROBOT_HOME_ROT = np.eye(3)  # 기본 회전(변화 없음)

# VR의 이동량을 로봇에 얼마나 반영할지 결정 (1.0 = 1:1 이동, 0.5 = 50% 축소 이동)
SCALE_FACTOR = 0.8

# ============================================================
# 2. 로봇 및 VR 초기화
# ============================================================
print("[시스템] 로봇 초기화 중...")
robot_config = SO101FollowerConfig(id=FOLLOWER_ARM_ID, port=FOLLOWER_ARM_PORT)
robot = SO101Follower(robot_config)
robot.connect()
my_chain = Chain.from_urdf_file(URDF_PATH)

print("[시스템] SteamVR 초기화 중...")
vr_system = openvr.init(openvr.VRApplication_Background)


def get_right_controller_idx(vr_system):
    """우측 컨트롤러의 인덱스를 반환"""
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if vr_system.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_Controller:
            if vr_system.getControllerRoleForTrackedDeviceIndex(i) == openvr.TrackedControllerRole_RightHand:
                return i
    return None


def extract_transform(pose_matrix):
    """OpenVR 3x4 행렬에서 3x3 회전 행렬과 3x1 위치 벡터 추출"""
    rot = np.array([
        [pose_matrix[0][0], pose_matrix[0][1], pose_matrix[0][2]],
        [pose_matrix[1][0], pose_matrix[1][1], pose_matrix[1][2]],
        [pose_matrix[2][0], pose_matrix[2][1], pose_matrix[2][2]]
    ])
    pos = np.array([pose_matrix[0][3], pose_matrix[1][3], pose_matrix[2][3]])
    return pos, rot


# ============================================================
# 3. 실시간 제어 루프
# ============================================================
vr_home_pos = None
vr_home_rot_inv = None

# 스무딩(Low-pass filter)용 이전 상태 저장 변수
prev_action = None
SMOOTH_ALPHA = 0.4  # 0.0(느림/부드러움) ~ 1.0(빠름/거침)

try:
    print("\n==================================================")
    print(" 컨트롤러를 편한 위치에 들고 대기하세요. (영점 조절)")
    for i in range(3, 0, -1):
        print(f" {i}초 전...")
        time.sleep(1)

    print("\n[동기화 완료] 실시간 로봇 추종을 시작합니다! (종료: Ctrl+C)")
    print("==================================================\n")

    while True:
        right_idx = get_right_controller_idx(vr_system)
        if right_idx is None:
            print("[경고] 우측 컨트롤러를 찾을 수 없습니다.", end="\r")
            time.sleep(0.1)
            continue

        poses = vr_system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                          openvr.k_unMaxTrackedDeviceCount)

        if poses[right_idx].bPoseIsValid:
            mat = poses[right_idx].mDeviceToAbsoluteTracking
            vr_pos, vr_rot = extract_transform(mat)

            # 1. 영점(Home) 세팅: 최초 1회 기준점 캡처
            if vr_home_pos is None:
                vr_home_pos = vr_pos
                vr_home_rot_inv = np.linalg.inv(vr_rot)  # 회전 기준점
                continue

            # 2. 이동량(Delta) 계산 및 좌표계 변환 (VR -> Robot)
            delta_vr_pos = vr_pos - vr_home_pos

            # [좌표계 변환 핵심] VR(Y가 위) -> 로봇(Z가 위, X가 정면)
            # VR의 Z축(앞뒤) -> 로봇의 X축
            # VR의 X축(좌우) -> 로봇의 Y축
            # VR의 Y축(상하) -> 로봇의 Z축
            target_pos = ROBOT_HOME_POS + np.array([
                -delta_vr_pos[2] * SCALE_FACTOR,  # Robot X (Forward)
                -delta_vr_pos[0] * SCALE_FACTOR,  # Robot Y (Left)
                delta_vr_pos[1] * SCALE_FACTOR  # Robot Z (Up)
            ])

            # 3. 회전량 계산 및 적용 (기본 회전 x 컨트롤러의 회전 변화량)
            # 로봇 URDF의 End-effector 기준에 맞춰 축 변환 행렬이 필요할 수 있습니다.
            delta_rot = vr_home_rot_inv @ vr_rot

            # 임시로 VR의 회전 변화를 로봇 기저(Base) 좌표계에 맞게 축 변환
            T_map = np.array([
                [0, 0, -1],
                [-1, 0, 0],
                [0, 1, 0]
            ])
            mapped_delta_rot = T_map @ delta_rot @ T_map.T
            target_rot = ROBOT_HOME_ROT @ mapped_delta_rot

            # 4. 역기구학(IK) 계산 (위치와 회전 동시 적용)
            try:
                # orientation_mode="all"을 통해 3x3 회전 행렬을 적용
                joint_angles = my_chain.inverse_kinematics(
                    target_position=target_pos,
                    target_orientation=target_rot,
                    orientation_mode="all"
                )
            except Exception as e:
                # 계산 불가능한 각도로 비틀었을 때 무시
                continue

            calculated_joints = joint_angles[1:]
            calculated_joints_deg = np.degrees(calculated_joints)

            # 5. 모터 딕셔너리 포장 및 스무딩 적용
            action_keys = [k for k in robot.action_features.keys() if k.endswith(".pos")]
            action = {}
            for i, key in enumerate(action_keys):
                raw_deg = float(calculated_joints_deg[i]) if i < len(calculated_joints_deg) else 0.0

                # 스무딩 필터: 급격한 튀어오름 방지 (안전 장치)
                if prev_action is not None:
                    smoothed_deg = (SMOOTH_ALPHA * raw_deg) + ((1.0 - SMOOTH_ALPHA) * prev_action[key])
                else:
                    smoothed_deg = raw_deg

                action[key] = smoothed_deg

            # 6. 로봇으로 전송
            robot.send_action(action)
            prev_action = action

            print(f"\r[실시간 추종 중] 로봇 X:{target_pos[0]:.2f} Y:{target_pos[1]:.2f} Z:{target_pos[2]:.2f}", end="")

        # 30Hz~50Hz로 통신 제한 (과부하 방지)
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n[시스템] 모니터링을 안전하게 종료합니다.")
finally:
    robot.disconnect()
    openvr.shutdown()