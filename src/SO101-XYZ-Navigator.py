import numpy as np
import time  # 시간 지연을 위한 모듈 추가
from ikpy.chain import Chain
from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower

# 1. 설정
FOLLOWER_ARM_ID = "skm_right_follower"
FOLLOWER_ARM_PORT = "/dev/ttyACM1"
URDF_PATH = "/home/roboseasy/shin_ws/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"

# 2. 로봇 초기화
robot_config = SO101FollowerConfig(id=FOLLOWER_ARM_ID, port=FOLLOWER_ARM_PORT)
robot = SO101Follower(robot_config)
robot.connect()

# 3. 역기구학(IK) 체인 불러오기
my_chain = Chain.from_urdf_file(URDF_PATH)


def move_to_xyz(x, y, z):
    """
    x, y, z 좌표(미터 단위)를 받아 로봇을 이동시킵니다.
    """
    target_vector = [x, y, z]

    # 1. 역기구학 계산 (결과는 라디안)
    joint_angles = my_chain.inverse_kinematics(target_position=target_vector)

    # 2. 베이스(0번 인덱스)를 제외한 실제 활성 관절 각도 추출
    calculated_joints = joint_angles[1:]

    # ⭐ 핵심 수정 1: 라디안(Radian)을 도(Degree) 단위로 변환
    calculated_joints_deg = np.degrees(calculated_joints)

    # 3. lerobot이 요구하는 모터 포지션 키 리스트 동적 추출
    action_keys = [k for k in robot.action_features.keys() if k.endswith(".pos")]

    # 4. 6개의 각도 데이터와 모터 이름을 딕셔너리로 맵핑
    action = {}
    for i, key in enumerate(action_keys):
        if i < len(calculated_joints_deg):
            # 변환된 Degree 값을 딕셔너리에 넣음
            action[key] = float(calculated_joints_deg[i])
        else:
            action[key] = 0.0

    # 5. 최종 조립된 딕셔너리 전송
    robot.send_action(action)
    print(f"\n[성공] 목표 좌표: {target_vector}")
    print(f"-> 전송된 액션(Degree): {action}")

    # ⭐ 핵심 수정 2: 로봇이 물리적으로 이동할 시간을 벌어줌 (2초 대기)
    print("로봇이 이동 중입니다... (10초 대기)")
    time.sleep(10)


# 사용 예시
try:
    print("좌표 제어 시작 (종료: Ctrl+C)")
    # 예: x=0.5m, y=0.5m, z=0.5m 지점으로 이동 명령
    move_to_xyz(0.2, 0., 0.1)

except KeyboardInterrupt:
    print("\n연결을 안전하게 종료합니다.")
finally:
    # 프로그램이 종료될 때 통신을 안전하게 닫아줌
    robot.disconnect()