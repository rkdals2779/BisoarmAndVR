import numpy as np
import time
import openvr
from ikpy.chain import Chain
from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower

# ============================================================
# 1. 환경 설정 및 스무딩(부드러움) 튜닝 파라미터
# ============================================================
FOLLOWER_ARM_ID = "skm_right_follower"
FOLLOWER_ARM_PORT = "/dev/ttyACM1"
URDF_PATH = "/home/roboseasy/shin_ws/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"

ROBOT_HOME_POS = np.array([0.2, 0.0, 0.1])
SCALE_FACTOR = 0.8

# ⭐ [핵심 튜닝 값] 숫자가 작을수록 묵직하고 부드러워지지만, 딜레이가 길어집니다. (0.01 ~ 1.0)
POS_SMOOTH_ALPHA = 0.1  # 1차: 목표 좌표가 이동하는 속도 (손떨림 방지)
JOINT_SMOOTH_ALPHA = 0.5  # 2차: 실제 모터가 회전하는 속도 (관절 부드러움)

# ⭐ [안전 장치] 모터가 0.02초(1프레임)당 움직일 수 있는 최대 각도 (너무 빠르면 로봇이 부서지거나 튐)
MAX_DEG_PER_FRAME = 2.0

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
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if vr_system.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_Controller:
            if vr_system.getControllerRoleForTrackedDeviceIndex(i) == openvr.TrackedControllerRole_RightHand:
                return i
    return None


def extract_position(pose_matrix):
    return np.array([pose_matrix[0][3], pose_matrix[1][3], pose_matrix[2][3]])


# ============================================================
# 3. 실시간 제어 루프
# ============================================================
vr_home_pos = None
prev_target_pos = None  # 좌표 스무딩용 이전 위치
prev_action = None  # 모터 스무딩용 이전 각도

try:
    print("\n==================================================")
    print(" 컨트롤러를 편한 위치에 들고 대기하세요. (영점 조절)")
    for i in range(3, 0, -1):
        print(f" {i}초 전...")
        time.sleep(1)

    print("\n[동기화 완료] 매우 부드러운 로봇 추종을 시작합니다! (종료: Ctrl+C)")
    print("==================================================\n")

    while True:
        right_idx = get_right_controller_idx(vr_system)
        if right_idx is None:
            time.sleep(0.1)
            continue

        poses = vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)

        if poses[right_idx].bPoseIsValid:
            mat = poses[right_idx].mDeviceToAbsoluteTracking
            vr_pos = extract_position(mat)

            if vr_home_pos is None:
                vr_home_pos = vr_pos
                continue

            # 1. 원본 목표 위치 계산
            delta_vr_pos = vr_pos - vr_home_pos
            raw_target_pos = ROBOT_HOME_POS + np.array([
                -delta_vr_pos[2] * SCALE_FACTOR,
                -delta_vr_pos[0] * SCALE_FACTOR,
                delta_vr_pos[1] * SCALE_FACTOR
            ])

            # ⭐ [추가됨] 1차 스무딩: XYZ 좌표 궤적 자체를 고무줄처럼 부드럽게 만들기
            if prev_target_pos is None:
                target_pos = raw_target_pos
            else:
                target_pos = (POS_SMOOTH_ALPHA * raw_target_pos) + ((1.0 - POS_SMOOTH_ALPHA) * prev_target_pos)
            prev_target_pos = target_pos

            # 2. 역기구학(IK) 계산
            try:
                joint_angles = my_chain.inverse_kinematics(target_position=target_pos)
            except Exception:
                continue

            calculated_joints_deg = np.degrees(joint_angles[1:])

            # 3. 모터 딕셔너리 포장 및 2차 스무딩 + 속도 제한
            action_keys = [k for k in robot.action_features.keys() if k.endswith(".pos")]
            action = {}

            for i, key in enumerate(action_keys):
                raw_deg = float(calculated_joints_deg[i]) if i < len(calculated_joints_deg) else 0.0

                if prev_action is not None and key in prev_action:
                    prev_deg = prev_action[key]

                    # ⭐ [추가됨] 최대 속도(각도) 제한: 갑자기 팍 꺾이는 현상 방지
                    delta_deg = raw_deg - prev_deg
                    if delta_deg > MAX_DEG_PER_FRAME:
                        raw_deg = prev_deg + MAX_DEG_PER_FRAME
                    elif delta_deg < -MAX_DEG_PER_FRAME:
                        raw_deg = prev_deg - MAX_DEG_PER_FRAME

                    # ⭐ 2차 스무딩: 최종 모터 각도를 한 번 더 부드럽게 깎아줌
                    smoothed_deg = (JOINT_SMOOTH_ALPHA * raw_deg) + ((1.0 - JOINT_SMOOTH_ALPHA) * prev_deg)
                else:
                    smoothed_deg = raw_deg

                action[key] = smoothed_deg

            # 4. 로봇으로 전송
            robot.send_action(action)
            prev_action = action

            print(f"\r[스무딩 추종 중] 로봇 위치 XYZ: [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]    ",
                  end="", flush=True)

        time.sleep(0.02)  # 50Hz 제어

except KeyboardInterrupt:
    print("\n\n[시스템] 모니터링을 안전하게 종료합니다.")
finally:
    robot.disconnect()
    openvr.shutdown()