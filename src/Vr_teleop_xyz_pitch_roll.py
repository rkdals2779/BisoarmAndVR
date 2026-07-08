"""
VR Teleoperation Controller for SO-101 Follower Arm
(위치 3DOF는 IK, 손목 pitch/roll 2DOF는 직접 관절 매핑, yaw 미사용)
=====================================================================

VR 컨트롤러의 위치(XYZ)는 IK로 풀고, 회전은 pitch/roll만 뽑아서
손목 관절(wrist_flex, wrist_roll)에 "직접" 매핑합니다. 컨트롤러의
orientation 전체를 IK의 target_orientation에 넣으면 5DOF 체인에
여유자유도가 생겨서 프레임마다 해가 다른 쪽으로 튀며 매우 불안정하게
움직입니다. 대신 물리적으로 대응되는 관절에 1:1로 직접 매핑하는
훨씬 안정적이고 예측 가능한 방식을 사용합니다.

    pitch (컨트롤러를 위아래로 끄덕이는 회전) -> wrist_flex (motor 4)
    roll  (컨트롤러를 손목처럼 비트는 회전)    -> wrist_roll (motor 5)
    yaw   (컨트롤러를 좌우로 젓는 회전)        -> 사용하지 않음
                                               (shoulder_pan이 좌우 위치를 담당)

파이프라인:
    VR 컨트롤러 위치(XYZ)
        -> One Euro Filter (적응형 스무딩)
        -> Inverse Kinematics - 위치만, 3DOF(shoulder_pan/lift, elbow_flex)
           (wrist_flex/wrist_roll은 active_links_mask로 IK 최적화 대상에서
            제외하고, 아래에서 계산한 pitch/roll 값으로 고정한 채 시딩
            -> 순기구학 계산에는 반영되지만 최적화되지는 않음)
    VR 컨트롤러 회전(pitch, roll)
        -> One Euro Filter (적응형 스무딩, 위치용과 별도 파라미터)
        -> wrist_flex / wrist_roll에 직접 각도로 매핑 (스케일/부호/한계 적용)
    (위치 3DOF + 손목 2DOF) 5개 관절
        -> 임계감쇠(critically damped) 관절 궤적 컨트롤러
           (실제 시간 단위로 속도/가속도 제한 -> 프레임 드랍에도 안전)
        -> 서보 명령 전송

그리퍼는 IK/회전 매핑과 완전히 분리해서 VR 트리거로 직접 제어합니다.
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation
import openvr
from ikpy.chain import Chain
from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower


# ============================================================
# 1. 설정값
# ============================================================
FOLLOWER_ARM_ID = "skm_right_follower"
FOLLOWER_ARM_PORT = "/dev/ttyACM1"
URDF_PATH = "/home/roboseasy/shin_ws/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"

ROBOT_HOME_POS = np.array([0.2, 0.0, 0.1])
SCALE_FACTOR = 0.8
CONTROL_HZ = 50
DT_NOMINAL = 1.0 / CONTROL_HZ

# --- One Euro Filter (Cartesian 위치 타겟 스무딩) ---
# min_cutoff: 손이 멈춰있을 때 기본 스무딩 강도 (낮을수록 부드러움)
# beta: 손이 빠르게 움직일 때 필터를 얼마나 "풀어주는지" (높을수록 반응 빠름/지연 적음)
ONEEURO_MIN_CUTOFF = 0.8
ONEEURO_BETA = 0.4
ONEEURO_D_CUTOFF = 1.0

# --- IK ---
IK_SKIP_THRESHOLD_M = 0.001   # 이 값보다 적게 움직이면 위치 IK 재계산 생략 (1mm)

# --- 관절 궤적 컨트롤러 (임계감쇠 2차 필터) ---
JOINT_KP = 45.0                 # 반응성 (높을수록 빠르지만 서보가 못 따라가면 진동 위험)
MAX_JOINT_SPEED_DEG_S = 120.0   # deg/s - 실제 로봇에서 보수적으로 시작해서 올릴 것
MAX_JOINT_ACCEL_DEG_S2 = 600.0  # deg/s^2

# --- 그리퍼 (IK/회전 매핑과 무관하게 VR 트리거로 직접 제어) ---
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

# --- 손목 회전 매핑 (pitch -> wrist_flex, roll -> wrist_roll, yaw 미사용) ---
# IK를 거치지 않고 컨트롤러 회전에서 뽑아낸 pitch/roll을 해당 관절에
# "직접" 더해서 명령합니다. 반대로 움직이면 아래 *_SIGN 값을 -1.0으로
# 뒤집으세요. 실행 중 상태줄에 값이 출력되니 그걸 보면서 튜닝하면 됩니다.
WRIST_FLEX_KEY = "wrist_flex.pos"
WRIST_ROLL_KEY = "wrist_roll.pos"

ROT_SCALE_FACTOR = 1.0   # 컨트롤러 회전각 -> 관절 각도 배율 (1.0 = 1:1)
PITCH_SIGN = -1.0          # wrist_flex 반대로 움직이면 -1.0
ROLL_SIGN = 1.0           # wrist_roll 반대로 움직이면 -1.0

# 회전 신호용 One Euro Filter (위치용과 신호 스케일이 달라 파라미터 분리)
ROT_ONEEURO_MIN_CUTOFF = 1.0
ROT_ONEEURO_BETA = 0.3
ROT_ONEEURO_D_CUTOFF = 1.0


# ============================================================
# 2. One Euro Filter
# ============================================================
class OneEuroFilter:
    """
    Casiez et al. 2012 1-euro filter.
    적응형 저역통과 필터: 느릴 땐 강하게 스무딩, 빠를 땐 지연을 최소화.
    numpy 벡터 전체에 대해 하나의 적응형 컷오프(속력 기반)를 공유해서 적용.
    위치(XYZ)뿐 아니라 임의 차원 벡터(여기서는 [pitch, roll])에도 그대로 씁니다.
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
    프로파일이 자연스럽게 나옵니다. wrist_flex/wrist_roll도 이 컨트롤러를
    그대로 통과하므로, IK 없이 직접 매핑하더라도 갑자기 튀지 않고
    부드럽게 움직입니다.
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


def extract_rotation_matrix(pose_matrix):
    """pyopenvr의 HmdMatrix34_t(3x4)에서 회전 성분(3x3)만 뽑아냅니다."""
    return np.array([
        [pose_matrix[0][0], pose_matrix[0][1], pose_matrix[0][2]],
        [pose_matrix[1][0], pose_matrix[1][1], pose_matrix[1][2]],
        [pose_matrix[2][0], pose_matrix[2][1], pose_matrix[2][2]],
    ])


def extract_pitch_roll(rel_rot_matrix):
    """
    캘리브레이션 시점 대비 상대 회전행렬에서 pitch/roll만 추출합니다.
    OpenVR 컨트롤러 로컬 축 관례: X=오른쪽, Y=위, Z=컨트롤러 뒤쪽
    (즉 -Z가 실제로 컨트롤러가 가리키는 포인팅 방향).

    'XZY' 순서(로컬 X축 회전=pitch 먼저, 그다음 로컬 Z축 회전=roll,
    마지막 로컬 Y축 회전=yaw)로 분해해서 yaw 성분을 분리해 버립니다.
    이렇게 하면 좌우로 손목을 젓는 동작(yaw)이 pitch/roll 값에
    거의 섞여 들어오지 않습니다.

    주의(짐벌락): roll이 ±90도 근처로 가면 pitch 추출이 흔들릴 수
    있습니다. 일반적인 손목 가동범위에서는 문제없지만, 실기에서 이상
    동작이 보이면 상태줄에 출력되는 pitch/roll 값을 먼저 확인하세요.
    """
    pitch, roll, _yaw = ScipyRotation.from_matrix(rel_rot_matrix).as_euler("XZY", degrees=False)
    return float(pitch), float(roll)


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

# --- 위치 전용 IK 마스크: wrist_flex/wrist_roll은 최적화 대상에서 제외 ---
# 이 두 관절은 아래 메인 루프에서 컨트롤러 pitch/roll로 "직접" 계산해서
# seed 값으로 고정합니다. active_links_mask에서 False로 두면 ikpy는 이
# 두 관절을 최적화하지 않고 seed에 넣어준 값을 그대로(순기구학 계산에는
# 반영하면서) 유지합니다 -> 위치 IK는 3DOF(shoulder_pan/lift, elbow_flex)만
# 풀게 되어 손목 회전과 절대 간섭하지 않습니다.
wrist_flex_arm_idx = ARM_JOINT_KEYS.index(WRIST_FLEX_KEY)
wrist_roll_arm_idx = ARM_JOINT_KEYS.index(WRIST_ROLL_KEY)
wrist_flex_full_idx = int(active_indices[wrist_flex_arm_idx])
wrist_roll_full_idx = int(active_indices[wrist_roll_arm_idx])

position_only_active_mask = active_mask.copy()
position_only_active_mask[wrist_flex_full_idx] = False
position_only_active_mask[wrist_roll_full_idx] = False
n_position_active = int(position_only_active_mask.sum())

print(
    "[진단] 위치 전용 IK 활성 관절 수:", n_position_active,
    "(wrist_flex/wrist_roll 제외, 3이어야 정상)"
)
assert n_position_active == 3, (
    "위치 전용 IK 활성 관절이 3개가 아닙니다. ARM_JOINT_KEYS 순서 또는 "
    "WRIST_FLEX_KEY/WRIST_ROLL_KEY 설정을 확인하세요."
)

# 이후 위치 IK 호출에서는 항상 이 마스크를 사용 (루프 안에서 매번 다시
# 설정할 필요 없이 한 번만 지정 - 이 스크립트에서는 이 마스크만 씀)
my_chain.active_links_mask = position_only_active_mask

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


# 손목 관절 한계(deg) - 컨트롤러 pitch/roll로 직접 계산한 목표각을
# 여기 안으로 clip해서 물리적 한계를 절대 넘지 않게 합니다.
wrist_flex_lo_deg = float(np.degrees(lower_bounds_full[wrist_flex_full_idx]))
wrist_flex_hi_deg = float(np.degrees(upper_bounds_full[wrist_flex_full_idx]))
wrist_roll_lo_deg = float(np.degrees(lower_bounds_full[wrist_roll_full_idx]))
wrist_roll_hi_deg = float(np.degrees(upper_bounds_full[wrist_roll_full_idx]))


print("[시스템] SteamVR 초기화 중...")
vr_system = openvr.init(openvr.VRApplication_Background)

pos_filter = OneEuroFilter(
    min_cutoff=ONEEURO_MIN_CUTOFF, beta=ONEEURO_BETA, d_cutoff=ONEEURO_D_CUTOFF
)
rot_filter = OneEuroFilter(
    min_cutoff=ROT_ONEEURO_MIN_CUTOFF, beta=ROT_ONEEURO_BETA, d_cutoff=ROT_ONEEURO_D_CUTOFF
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

# 손목 회전의 "영점"도 로봇의 실제 현재 각도로 잡습니다. 컨트롤러 pitch/roll
# 델타가 여기에 더해지므로, 시작하는 순간에는 델타가 0이라 로봇이 절대
# 튀지 않습니다.
wrist_flex_home_deg = float(initial_arm_deg[wrist_flex_arm_idx])
wrist_roll_home_deg = float(initial_arm_deg[wrist_roll_arm_idx])

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
vr_home_rot = None
prev_target_pos = None
last_loop_t = time.perf_counter()

try:
    print("\n==================================================")
    print(" 컨트롤러를 편한 위치와 방향으로 들고 대기하세요. (영점 조절)")
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
        vr_rot = extract_rotation_matrix(mat)

        if vr_home_pos is None:
            vr_home_pos = vr_pos
            vr_home_rot = vr_rot
            continue

        # 1. VR delta 위치 -> 로봇 좌표계 목표 위치
        delta_vr_pos = vr_pos - vr_home_pos
        raw_target_pos = ROBOT_HOME_POS + np.array([
            -delta_vr_pos[2] * SCALE_FACTOR,
            -delta_vr_pos[0] * SCALE_FACTOR,
            delta_vr_pos[1] * SCALE_FACTOR,
        ])

        # 2. One Euro Filter (위치 - 적응형 스무딩)
        target_pos = pos_filter.filter(raw_target_pos, loop_start)

        # 2b. VR delta 회전 -> pitch/roll (yaw는 버림) -> One Euro Filter
        #     IK를 거치지 않고 wrist_flex/wrist_roll에 직접 매핑합니다.
        rel_rot = vr_home_rot.T @ vr_rot
        pitch_raw, roll_raw = extract_pitch_roll(rel_rot)
        pitch_filt, roll_filt = rot_filter.filter(np.array([pitch_raw, roll_raw]), loop_start)

        wrist_flex_target_deg = wrist_flex_home_deg + PITCH_SIGN * np.degrees(pitch_filt) * ROT_SCALE_FACTOR
        wrist_roll_target_deg = wrist_roll_home_deg + ROLL_SIGN * np.degrees(roll_filt) * ROT_SCALE_FACTOR
        wrist_flex_target_deg = float(np.clip(wrist_flex_target_deg, wrist_flex_lo_deg, wrist_flex_hi_deg))
        wrist_roll_target_deg = float(np.clip(wrist_roll_target_deg, wrist_roll_lo_deg, wrist_roll_hi_deg))

        # 3. IK - 위치만(3DOF). 실제로 움직였을 때만 재계산, 항상 이전 해로 시딩.
        #    wrist_flex/wrist_roll은 seed에 "이번 프레임의 직접 계산값"을 넣어서
        #    순기구학에는 반영되지만(팔 길이/오프셋 효과), active_links_mask가
        #    False라서 최적화 대상은 아닙니다.
        need_ik = (
            prev_target_pos is None
            or np.linalg.norm(target_pos - prev_target_pos) > IK_SKIP_THRESHOLD_M
        )

        # seed를 매번 물리적 관절 한계 안으로 clip (아래 설명 참고)
        seed = clip_to_joint_bounds(prev_ik_solution_full)
        seed[wrist_flex_full_idx] = np.radians(wrist_flex_target_deg)
        seed[wrist_roll_full_idx] = np.radians(wrist_roll_target_deg)

        if need_ik:
            try:
                joint_angles_full = my_chain.inverse_kinematics(
                    target_position=target_pos,
                    initial_position=seed,
                )
                joint_angles_full = clip_to_joint_bounds(joint_angles_full)
            except Exception as e:
                print(f"\n[경고] IK 실패, 이전 관절각 유지: {e}")
                joint_angles_full = seed
        else:
            joint_angles_full = seed

        # wrist_flex/wrist_roll은 IK와 완전히 무관하게, 매 프레임 항상 방금
        # 계산한 값으로 덮어씁니다. (position_only_active_mask 하에서는
        # 어차피 IK가 건드리지 않지만, 방어적으로 명시해 둡니다. 이렇게 하면
        # need_ik가 False라서 위치 IK를 건너뛴 프레임에도 손목 회전만은
        # 매번 새로 갱신됩니다.)
        joint_angles_full[wrist_flex_full_idx] = np.radians(wrist_flex_target_deg)
        joint_angles_full[wrist_roll_full_idx] = np.radians(wrist_roll_target_deg)

        prev_ik_solution_full = joint_angles_full
        prev_target_pos = target_pos

        arm_target_deg = np.degrees(np.asarray(joint_angles_full)[active_mask])

        # 4. 임계감쇠 궤적 컨트롤러 -> 속도/가속도가 제한된 매끄러운 움직임
        #    (위치 3DOF + 손목 pitch/roll 2DOF 모두 여기를 통과하므로
        #     손목이 갑자기 튀지 않고 부드럽게 따라옵니다.)
        arm_commanded_deg = trajectory.update(arm_target_deg, dt)

        # 5. 그리퍼 - IK/회전 매핑과 무관하게 트리거로 직접 제어
        trigger = get_trigger_value(vr_system, right_idx)
        gripper_deg = GRIPPER_OPEN_DEG + trigger * (GRIPPER_CLOSE_DEG - GRIPPER_OPEN_DEG)

        # 6. 패킹 및 전송
        action = {key: float(arm_commanded_deg[i]) for i, key in enumerate(ARM_JOINT_KEYS)}
        action[GRIPPER_KEY] = float(gripper_deg)
        robot.send_action(action)

        print(
            f"\r[추종 중] XYZ: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]  "
            f"flex(4): {wrist_flex_target_deg:6.1f}  roll(5): {wrist_roll_target_deg:6.1f}  "
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