import numpy as np
import time
import openvr
from ikpy.chain import Chain
from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower


# ============================================================
# 0. One Euro Filter 클래스 (전문가 추천 )
# ============================================================
class OneEuroFilter:
    def __init__(self, freq, mincutoff=1.0, beta=0.0, dcutoff=1.0):
        self.freq = freq
        self.mincutoff = mincutoff
        self.beta = beta
        self.dcutoff = dcutoff
        self.x_prev = None
        self.dx_prev = None

    def _alpha(self, cutoff):
        tau = 1.0 / (2 * np.pi * cutoff)
        te = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def __call__(self, x, dt):
        if self.x_prev is None:
            self.x_prev = x
            self.dx_prev = np.zeros_like(x)
            return x

        # 속도 추정
        dx = (x - self.x_prev) / dt
        edx = self.beta * self.dcutoff + (1 - self.beta) * self._alpha(self.dcutoff)  # 간단 구현
        dx_hat = edx * dx + (1 - edx) * self.dx_prev

        # 컷오프 조정
        cutoff = self.mincutoff + self.beta * np.abs(dx_hat)
        alpha = self._alpha(cutoff)

        # 결과값
        x_hat = alpha * x + (1 - alpha) * self.x_prev
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat


# ============================================================
# 1. 환경 설정 및 파라미터
# ============================================================
# ... 기존 설정 동일 ...
FOLLOWER_ARM_ID = "skm_right_follower"
FOLLOWER_ARM_PORT = "/dev/ttyACM1"
URDF_PATH = "/home/roboseasy/shin_ws/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"
SCALE_FACTOR = 0.8
ROBOT_HOME_POS = np.array([0.2, 0.0, 0.1])

# 제어 파라미터
SPRING_K = 0.2  # Workspace Spring 강성
MAX_VEL_DEG = 90.0  # deg/sec

# 초기화
robot_config = SO101FollowerConfig(id=FOLLOWER_ARM_ID, port=FOLLOWER_ARM_PORT)
robot = SO101Follower(robot_config)
robot.connect()
my_chain = Chain.from_urdf_file(URDF_PATH)
vr_system = openvr.init(openvr.VRApplication_Background)

# 필터 및 상태 변수
euro_filter = OneEuroFilter(freq=50, mincutoff=1.0, beta=0.01)
current_joints = np.zeros(my_chain.length)  # 초기 관절 상태
prev_time = time.time()

# ============================================================
# 2. 메인 루프
# ============================================================
try:
    print("[시스템] 고도화된 제어 모드 가동 중...")

    while True:
        # 1. VR Pose 읽기
        # ... (get_right_controller_idx 생략) ...
        # [raw_pos 획득 과정]

        dt = time.time() - prev_time
        prev_time = time.time()

        # 2. One Euro Filter 적용 [cite: 30]
        filtered_pos = euro_filter(raw_pos, dt)

        # 3. Workspace Spring 적용
        # 로봇의 현재 위치(FK)를 가져와서 목표와의 오차만큼만 움직임
        current_ee_pos = my_chain.forward_kinematics(current_joints)[:3, 3]
        target_vel = SPRING_K * (filtered_pos - current_ee_pos)

        # 4. Differential IK (Jacobian)
        # J * dq = dx => dq = J_pinv * dx
        jacobian = my_chain.jacobian(current_joints)
        jacobian_pos = jacobian[:3, :]  # 위치 제어용 Jacobian 행렬

        try:
            j_pinv = np.linalg.pinv(jacobian_pos)
            delta_joints = j_pinv @ target_vel

            # 5. 속도 제한 적용 (deg/s 기준)
            max_delta = (MAX_VEL_DEG * dt) * (np.pi / 180.0)
            delta_joints = np.clip(delta_joints, -max_delta, max_delta)

            current_joints += delta_joints

        except Exception as e:
            continue

        # 6. 로봇 송신
        # current_joints를 딕셔너리로 변환하여 전송
        # ... (robot.send_action 구현) ...

        time.sleep(0.02)

except KeyboardInterrupt:
    robot.disconnect()
    openvr.shutdown()