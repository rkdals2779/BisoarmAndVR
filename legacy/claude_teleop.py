"""
VR Teleoperation Controller for SO-101 Follower Arm (위치 전용, XYZ만)
=====================================================================

회전(orientation) 제어를 완전히 뺀 버전입니다. VR 컨트롤러의 위치(XYZ)만
읽어서 로봇 팔을 움직입니다. 컨트롤러의 회전값은 그리퍼 트리거를 읽는 것
외에는 전혀 사용하지 않습니다.

파이프라인:
    VR 컨트롤러 위치(XYZ)
        -> One Euro Filter (적응형 스무딩)
        -> Inverse Kinematics - 위치만 (이전 해로 시딩 + active_links_mask 사용)
        -> 임계감쇠(critically damped) 관절 궤적 컨트롤러
           (실제 시간 단위로 속도/가속도 제한 -> 프레임 드랍에도 안전)
        -> 서보 명령 전송

그리퍼는 IK와 완전히 분리해서 VR 트리거로 직접 제어합니다.
"""

import time
import numpy as np
import openvr
from ikpy.chain import Chain
from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower


# ============================================================
# 1. 설정값
# ============================================================
FOLLOWER_ARM_ID = "skm_right_follower"
FOLLOWER_ARM_PORT = "/dev/ttyACM0"
URDF_PATH = "/home/roboseasy/shin_ws/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"

ROBOT_HOME_POS = np.array([0.2, 0.0, 0.1])
SCALE_FACTOR = 0.8
CONTROL_HZ = 50
DT_NOMINAL = 1.0 / CONTROL_HZ

# --- One Euro Filter (Cartesian 타겟 스무딩) ---
# min_cutoff: 손이 멈춰있을 때 기본 스무딩 강도 (낮을수록 부드러움)
# beta: 손이 빠르게 움직일 때 필터를 얼마나 "풀어주는지" (높을수록 반응 빠름/지연 적음)
ONEEURO_MIN_CUTOFF = 0.8
ONEEURO_BETA = 0.4
ONEEURO_D_CUTOFF = 1.0

# --- IK ---
IK_SKIP_THRESHOLD_M = 0.001   # 이 값보다 적게 움직이면 IK 재계산 생략 (1mm)

# --- 관절 궤적 컨트롤러 (임계감쇠 2차 필터) ---
JOINT_KP = 45.0                 # 반응성 (높을수록 빠르지만 서보가 못 따라가면 진동 위험)
MAX_JOINT_SPEED_DEG_S = 120.0   # deg/s - 실제 로봇에서 보수적으로 시작해서 올릴 것
MAX_JOINT_ACCEL_DEG_S2 = 600.0  # deg/s^2

# --- 그리퍼 (IK와 무관하게 VR 트리거로 직접 제어) ---
GRIPPER_KEY = "gripper.pos"
GRIPPER_OPEN_DEG = 90
GRIPPER_CLOSE_DEG = 0   # 실제 그리퍼 가동 범위를 확인 후 조정하세요

# IK 체인에서 위치 IK에 사용되는 활성 관절 순서라고 가정한 목록.
# 표준 SO-101 URDF 기준 가정이며, 아래 진단 출력으로 반드시 검증하세요.
ARM_JOINT_KEYS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
]


# ============================================================
# 2. One Euro Filter
# ============================================================
class OneEuroFilter:
    """
    Casiez et al. 2012 1-euro filter.
    적응형 저역통과 필터: 느릴 땐 강하게 스무딩, 빠를 땐 지연을 최소화.
    numpy 벡터 전체에 대해 하나의 적응형 컷오프(속력 기반)를 공유해서 적용.
    """

    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = None
        self.t_prev = None

    @staticmethod
    def _alpha(dt, cutoff):
        tau = 1.0 / (2 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def filter(self, x, t):
        x = np.asarray(x, dtype=float)

        if self.t_prev is None:
            self.x_prev = x.copy()
            self.dx_prev = np.zeros_like(x)
            self.t_prev = t
            return x.copy()

        dt = max(t - self.t_prev, 1e-6)

        dx = (x - self.x_prev) / dt
        a_d = self._alpha(dt, self.d_cutoff)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev

        speed = float(np.linalg.norm(dx_hat))
        cutoff = self.min_cutoff + self.beta * speed
        a = self._alpha(dt, cutoff)
        x_hat = a * x + (1 - a) * self.x_prev

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat.copy()


# ============================================================
# 3. 임계감쇠 관절 궤적 컨트롤러
# ============================================================
class JointTrajectoryController:
    """
    ~CONTROL_HZ로 들어오는 목표 관절각(deg) 스트림을, 속도/가속도가
    제한된 물리적으로 매끄러운 움직임으로 변환.

    관절마다 임계감쇠(zeta=1) 스프링-댐퍼를 적용:
        accel = kp * (target - pos) - kd * vel,   kd = 2*sqrt(kp)

    임계감쇠이므로 목표가 계속 움직여도 오버슈트/진동 없이 가속->감속
    프로파일이 자연스럽게 나옵니다.
    """

    def __init__(self, kp, max_vel_deg_s, max_acc_deg_s2):
        self.kp = kp
        self.kd = 2.0 * np.sqrt(kp)  # 임계감쇠 조건
        self.max_vel = max_vel_deg_s
        self.max_acc = max_acc_deg_s2
        self.pos = None
        self.vel = None

    def reset(self, pos_deg):
        self.pos = np.asarray(pos_deg, dtype=float).copy()
        self.vel = np.zeros_like(self.pos)

    def update(self, target_deg, dt):
        target_deg = np.asarray(target_deg, dtype=float)
        if self.pos is None:
            self.reset(target_deg)
            return self.pos.copy()

        error = target_deg - self.pos
        accel = self.kp * error - self.kd * self.vel
        accel = np.clip(accel, -self.max_acc, self.max_acc)

        self.vel = self.vel + accel * dt
        self.vel = np.clip(self.vel, -self.max_vel, self.max_vel)

        self.pos = self.pos + self.vel * dt
        return self.pos.copy()


# ============================================================
# 4. VR 헬퍼 함수
# ============================================================
def get_right_controller_idx(vr_system):
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if vr_system.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_Controller:
            if vr_system.getControllerRoleForTrackedDeviceIndex(i) == openvr.TrackedControllerRole_RightHand:
                return i
    return None


def extract_position(pose_matrix):
    return np.array([pose_matrix[0][3], pose_matrix[1][3], pose_matrix[2][3]])


def get_trigger_value(vr_system, controller_idx):
    """0.0(뗌) ~ 1.0(완전히 당김). pyopenvr 버전에 따라 axis 인덱스가
    다를 수 있으니 실제 컨트롤러로 값이 잘 들어오는지 먼저 확인하세요."""
    result, state = vr_system.getControllerState(controller_idx)
    if not result:
        return 0.0
    return float(state.rAxis[1].x)


# ============================================================
# 5. 로봇 및 VR 초기화
# ============================================================
print("[시스템] 로봇 초기화 중...")
robot_config = SO101FollowerConfig(id=FOLLOWER_ARM_ID, port=FOLLOWER_ARM_PORT)
robot = SO101Follower(robot_config)
robot.connect()
my_chain = Chain.from_urdf_file(URDF_PATH)

# ikpy가 자동 생성하는 active_links_mask는 이 URDF에서는 신뢰할 수 없습니다.
# (Base link, gripper_frame_joint가 URDF상 'fixed' 타입인데도 mask에는 True로
# 잡혀서, 실행 시 ikpy 자신이 "is of type 'fixed' but set as active" 경고를
# 띄웁니다.) 대신 각 링크의 실제 joint_type을 직접 확인해서 'fixed'가 아닌
# 것만 진짜 활성 관절(회전 모터)로 판단합니다.
joint_types = [getattr(link, "joint_type", "fixed") for link in my_chain.links]
active_mask = np.array([jt not in ("fixed", None) for jt in joint_types])
n_active = int(active_mask.sum())

print("[진단] IK 체인 링크:", [link.name for link in my_chain.links])
print("[진단] joint_type:", joint_types)
print("[진단] 실제 활성 관절 마스크:", active_mask, f"(활성 관절 수: {n_active})")

assert n_active == len(ARM_JOINT_KEYS), (
    f"체인의 활성 관절 수({n_active})가 ARM_JOINT_KEYS 길이({len(ARM_JOINT_KEYS)})와 다릅니다.\n"
    "위에 출력된 링크 목록을 보고 ARM_JOINT_KEYS 순서/개수를 실제 URDF에 맞게 수정하세요."
)

active_indices = np.where(active_mask)[0]

# --- 각 링크(관절)의 실제 물리적 한계(rad)를 URDF에서 추출 ---
# scipy의 least_squares는 initial guess(seed)가 bounds를 단 하나라도 벗어나면
# 최적화를 시작도 하지 않고 즉시 "Initial guess is outside of provided bounds"로
# 실패합니다. 그 실패한 seed가 다음 프레임에 그대로 재사용되면 영원히 같은 에러만
# 반복되므로, 매번 seed를 물리적 한계 안으로 clip해서 이 악순환을 끊습니다.
link_bounds = [getattr(link, "bounds", (None, None)) for link in my_chain.links]
lower_bounds_full = np.array([
    (b[0] if (b is not None and b[0] is not None) else -np.inf) for b in link_bounds
])
upper_bounds_full = np.array([
    (b[1] if (b is not None and b[1] is not None) else np.inf) for b in link_bounds
])


def clip_to_joint_bounds(full_angles_rad):
    return np.clip(full_angles_rad, lower_bounds_full, upper_bounds_full)


print("[시스템] SteamVR 초기화 중...")
vr_system = openvr.init(openvr.VRApplication_Background)

pos_filter = OneEuroFilter(
    min_cutoff=ONEEURO_MIN_CUTOFF, beta=ONEEURO_BETA, d_cutoff=ONEEURO_D_CUTOFF
)
trajectory = JointTrajectoryController(
    kp=JOINT_KP,
    max_vel_deg_s=MAX_JOINT_SPEED_DEG_S,
    max_acc_deg_s2=MAX_JOINT_ACCEL_DEG_S2,
)

# --- 궤적/IK 시드를 "로봇의 실제 현재 자세"로 초기화 (추측값이 아니라 관측값 사용) ---
obs = robot.get_observation()
initial_arm_deg = np.array([obs[k] for k in ARM_JOINT_KEYS])
trajectory.reset(initial_arm_deg)

print("[진단] 관절 한계(deg) vs 현재 각도:")
for key, idx, cur_deg in zip(ARM_JOINT_KEYS, active_indices, initial_arm_deg):
    lo = lower_bounds_full[idx]
    hi = upper_bounds_full[idx]
    lo_deg = np.degrees(lo) if np.isfinite(lo) else -np.inf
    hi_deg = np.degrees(hi) if np.isfinite(hi) else np.inf
    flag = " <-- 이미 한계 밖!" if not (lo_deg - 1e-6 <= cur_deg <= hi_deg + 1e-6) else ""
    print(f"    {key:20s}: [{lo_deg:8.1f}, {hi_deg:8.1f}]   현재: {cur_deg:8.1f}{flag}")

prev_ik_solution_full = np.zeros(len(my_chain.links))
prev_ik_solution_full[active_mask] = np.radians(initial_arm_deg)
prev_ik_solution_full = clip_to_joint_bounds(prev_ik_solution_full)


# ============================================================
# 6. 실시간 제어 루프
# ============================================================
vr_home_pos = None
prev_target_pos = None
last_loop_t = time.perf_counter()

try:
    print("\n==================================================")
    print(" 컨트롤러를 편한 위치에 들고 대기하세요. (영점 조절)")
    for i in range(3, 0, -1):
        print(f" {i}초 전...")
        time.sleep(1)

    print("\n[동기화 완료] 추종을 시작합니다! (종료: Ctrl+C)")
    print("==================================================\n")

    while True:
        loop_start = time.perf_counter()
        dt = max(loop_start - last_loop_t, 1e-3)
        last_loop_t = loop_start

        right_idx = get_right_controller_idx(vr_system)
        if right_idx is None:
            time.sleep(0.1)
            continue

        poses = vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )

        if not poses[right_idx].bPoseIsValid:
            time.sleep(DT_NOMINAL)
            continue

        mat = poses[right_idx].mDeviceToAbsoluteTracking
        vr_pos = extract_position(mat)

        if vr_home_pos is None:
            vr_home_pos = vr_pos
            continue

        # 1. VR delta -> 로봇 좌표계 목표 위치
        delta_vr_pos = vr_pos - vr_home_pos
        raw_target_pos = ROBOT_HOME_POS + np.array([
            -delta_vr_pos[2] * SCALE_FACTOR,
            -delta_vr_pos[0] * SCALE_FACTOR,
            delta_vr_pos[1] * SCALE_FACTOR,
        ])

        # 2. One Euro Filter (적응형 스무딩)
        target_pos = pos_filter.filter(raw_target_pos, loop_start)

        # 3. IK - 위치만. 실제로 움직였을 때만 재계산, 항상 이전 해로 시딩
        need_ik = (
            prev_target_pos is None
            or np.linalg.norm(target_pos - prev_target_pos) > IK_SKIP_THRESHOLD_M
        )

        if need_ik:
            # seed를 매번 물리적 관절 한계 안으로 clip (아래 설명 참고)
            seed = clip_to_joint_bounds(prev_ik_solution_full)
            try:
                joint_angles_full = my_chain.inverse_kinematics(
                    target_position=target_pos,
                    initial_position=seed,
                )
                joint_angles_full = clip_to_joint_bounds(joint_angles_full)
                prev_ik_solution_full = joint_angles_full
            except Exception as e:
                print(f"\n[경고] IK 실패, 이전 관절각 유지: {e}")
                joint_angles_full = prev_ik_solution_full
        else:
            joint_angles_full = prev_ik_solution_full

        prev_target_pos = target_pos

        arm_target_deg = np.degrees(np.asarray(joint_angles_full)[active_mask])

        # 4. 임계감쇠 궤적 컨트롤러 -> 속도/가속도가 제한된 매끄러운 움직임
        arm_commanded_deg = trajectory.update(arm_target_deg, dt)

        # 5. 그리퍼 - IK와 무관하게 트리거로 직접 제어
        trigger = get_trigger_value(vr_system, right_idx)
        gripper_deg = GRIPPER_OPEN_DEG + trigger * (GRIPPER_CLOSE_DEG - GRIPPER_OPEN_DEG)

        # 6. 패킹 및 전송
        action = {key: float(arm_commanded_deg[i]) for i, key in enumerate(ARM_JOINT_KEYS)}
        action[GRIPPER_KEY] = float(gripper_deg)
        robot.send_action(action)

        print(
            f"\r[추종 중] XYZ: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]  "
            f"grip: {gripper_deg:5.1f}    ",
            end="", flush=True,
        )

        elapsed = time.perf_counter() - loop_start
        time.sleep(max(0.0, DT_NOMINAL - elapsed))

except KeyboardInterrupt:
    print("\n\n[시스템] 안전하게 종료합니다.")
finally:
    robot.disconnect()
    openvr.shutdown()