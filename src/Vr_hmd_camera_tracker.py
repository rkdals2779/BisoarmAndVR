import time
import serial
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation
import openvr

# ============================================================
# 1. 설정값
# ============================================================
CAMERA_PORT = "/dev/ttyACM0"  # 카메라 모터가 연결된 USB-to-TTL 포트 (실제에 맞게 변경)
BAUDRATE = 1000000

PAN_MOTOR_ID = 7  # 카메라 좌우 도리도리 (Yaw)
TILT_MOTOR_ID = 8  # 카메라 상하 끄덕임 (Pitch)

# 모터의 기본(정면) 영점 (Feetech 12비트 엔코더 기준 2048 값 = 180도)
PAN_HOME_DEG = 180.0
TILT_HOME_DEG = 180.0

# 모터 가동 범위 (선 꼬임 방지 및 기구적 한계)
PAN_MIN_DEG, PAN_MAX_DEG = 90.0, 270.0
TILT_MIN_DEG, TILT_MAX_DEG = 110.0, 250.0

# HMD -> 모터 각도 매핑 및 방향 (반대로 돌면 부호를 -1.0으로 수정)
YAW_SCALE = 1.0  # 고개 좌우 회전 배율
PITCH_SCALE = -1.0  # 고개 상하 회전 배율
YAW_SIGN = -1.0  # Yaw 방향 반전
PITCH_SIGN = 1.0  # Pitch 방향 반전

CONTROL_HZ = 50
DT_NOMINAL = 1.0 / CONTROL_HZ

# 필터 및 제어기 파라미터 (원문 코드와 동일하게 유지)
ROT_ONEEURO_MIN_CUTOFF = 1.0
ROT_ONEEURO_BETA = 0.3
ROT_ONEEURO_D_CUTOFF = 1.0

JOINT_KP = 45.0
MAX_JOINT_SPEED_DEG_S = 150.0
MAX_JOINT_ACCEL_DEG_S2 = 600.0


# ============================================================
# 2. Feetech 모터 직접 제어기 (pyserial)
# ============================================================
class FeetechPanTiltController:
    """
    Feetech STS/SCS 시리즈를 직렬 프로토콜로 직접 제어.
    복잡한 SDK 없이 WRITE_DATA(0x03) 명령어로 즉각적인 위치 제어 수행.
    """

    def __init__(self, port, baudrate=1000000):
        self.ser = serial.Serial(port, baudrate, timeout=0.01)

    def write_position(self, motor_id, pos_deg):
        # 0~360도를 0~4095로 변환
        pos_counts = int(pos_deg * 4096 / 360.0)
        pos_counts = max(0, min(4095, pos_counts))

        pos_l = pos_counts & 0xFF
        pos_h = (pos_counts >> 8) & 0xFF

        # 패킷: [0xFF, 0xFF, ID, Length, Cmd(0x03), Addr(0x2A), PosL, PosH, TimeL, TimeH, SpeedL, SpeedH, Checksum]
        length = 9
        cmd = 0x03
        addr = 0x2A  # Target Position Register

        checksum = ~(motor_id + length + cmd + addr + pos_l + pos_h + 0 + 0 + 0 + 0) & 0xFF

        packet = bytearray([
            0xFF, 0xFF, motor_id, length, cmd, addr,
            pos_l, pos_h, 0x00, 0x00, 0x00, 0x00, checksum
        ])
        self.ser.write(packet)

    def release_torque(self, motor_id):
        """
        모터의 토크를 해제하여 손으로 굴릴 수 있는 Free 상태로 만듭니다.
        Torque Enable (Address 0x28) 값을 0으로 설정합니다.
        """
        length = 4
        cmd = 0x03
        addr = 0x28  # Torque Enable Register
        val = 0x00  # 0: Torque Off, 1: Torque On

        checksum = ~(motor_id + length + cmd + addr + val) & 0xFF

        packet = bytearray([
            0xFF, 0xFF, motor_id, length, cmd, addr, val, checksum
        ])
        self.ser.write(packet)

    def close(self):
        self.ser.close()


# ============================================================
# 3. 유틸리티 및 제어기 (OneEuro, Trajectory)
# ============================================================
class OneEuroFilter:
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
            self.x_prev, self.dx_prev, self.t_prev = x.copy(), np.zeros_like(x), t
            return x.copy()

        dt = max(t - self.t_prev, 1e-6)
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(dt, self.d_cutoff)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev

        speed = float(np.linalg.norm(dx_hat))
        cutoff = self.min_cutoff + self.beta * speed
        a = self._alpha(dt, cutoff)
        x_hat = a * x + (1 - a) * self.x_prev

        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat.copy()


class JointTrajectoryController:
    def __init__(self, kp, max_vel_deg_s, max_acc_deg_s2):
        self.kp = kp
        self.kd = 2.0 * np.sqrt(kp)
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
        accel = np.clip(self.kp * error - self.kd * self.vel, -self.max_acc, self.max_acc)
        self.vel = np.clip(self.vel + accel * dt, -self.max_vel, self.max_vel)
        self.pos = self.pos + self.vel * dt
        return self.pos.copy()


def extract_rotation_matrix(pose_matrix):
    return np.array([
        [pose_matrix[0][0], pose_matrix[0][1], pose_matrix[0][2]],
        [pose_matrix[1][0], pose_matrix[1][1], pose_matrix[1][2]],
        [pose_matrix[2][0], pose_matrix[2][1], pose_matrix[2][2]],
    ])


def extract_hmd_yaw_pitch(rel_rot_matrix):
    yaw, pitch, _roll = ScipyRotation.from_matrix(rel_rot_matrix).as_euler("YXZ", degrees=False)
    return float(yaw), float(pitch)


# ============================================================
# 4. 초기화 및 실시간 제어 루프
# ============================================================
print("[시스템] Feetech 카메라 모터 연결 중...")
motors = FeetechPanTiltController(CAMERA_PORT, BAUDRATE)

print("[시스템] SteamVR 초기화 중...")
vr_system = openvr.init(openvr.VRApplication_Background)
hmd_idx = openvr.k_unTrackedDeviceIndex_Hmd

rot_filter = OneEuroFilter(ROT_ONEEURO_MIN_CUTOFF, ROT_ONEEURO_BETA, ROT_ONEEURO_D_CUTOFF)
trajectory = JointTrajectoryController(JOINT_KP, MAX_JOINT_SPEED_DEG_S, MAX_JOINT_ACCEL_DEG_S2)

# 모터를 Home 포지션으로 리셋
trajectory.reset([PAN_HOME_DEG, TILT_HOME_DEG])
motors.write_position(PAN_MOTOR_ID, PAN_HOME_DEG)
motors.write_position(TILT_MOTOR_ID, TILT_HOME_DEG)

vr_home_rot = None
last_loop_t = time.perf_counter()

try:
    print("\n==================================================")
    print(" HMD(헤드셋)를 정면을 향해 편안하게 쓰고 대기하세요.")
    for i in range(3, 0, -1):
        print(f" {i}초 전...")
        time.sleep(1)

    # 영점 캘리브레이션 획득
    poses = vr_system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                      openvr.k_unMaxTrackedDeviceCount)
    if poses[hmd_idx].bPoseIsValid:
        vr_home_rot = extract_rotation_matrix(poses[hmd_idx].mDeviceToAbsoluteTracking)
        print("\n[동기화 완료] 카메라 시선 추종을 시작합니다! (종료: Ctrl+C)")
    else:
        print("[에러] HMD 포즈를 읽어오지 못했습니다. VR이 켜져 있는지 확인하세요.")
        exit()
    print("==================================================\n")

    while True:
        loop_start = time.perf_counter()
        dt = max(loop_start - last_loop_t, 1e-3)
        last_loop_t = loop_start

        poses = vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )

        if not poses[hmd_idx].bPoseIsValid:
            time.sleep(DT_NOMINAL)
            continue

        mat = poses[hmd_idx].mDeviceToAbsoluteTracking
        vr_rot = extract_rotation_matrix(mat)
        rel_rot = vr_home_rot.T @ vr_rot

        yaw_raw, pitch_raw = extract_hmd_yaw_pitch(rel_rot)
        yaw_filt, pitch_filt = rot_filter.filter(np.array([yaw_raw, pitch_raw]), loop_start)

        pan_target = PAN_HOME_DEG + YAW_SIGN * np.degrees(yaw_filt) * YAW_SCALE
        tilt_target = TILT_HOME_DEG + PITCH_SIGN * np.degrees(pitch_filt) * PITCH_SCALE

        pan_target = float(np.clip(pan_target, PAN_MIN_DEG, PAN_MAX_DEG))
        tilt_target = float(np.clip(tilt_target, TILT_MIN_DEG, TILT_MAX_DEG))

        smoothed_angles = trajectory.update([pan_target, tilt_target], dt)

        motors.write_position(PAN_MOTOR_ID, smoothed_angles[0])
        motors.write_position(TILT_MOTOR_ID, smoothed_angles[1])

        print(
            f"\r[카메라 추종] Pan: {smoothed_angles[0]:6.1f} 도 | Tilt: {smoothed_angles[1]:6.1f} 도   ",
            end="", flush=True
        )

        elapsed = time.perf_counter() - loop_start
        time.sleep(max(0.0, DT_NOMINAL - elapsed))

except KeyboardInterrupt:
    print("\n\n[시스템] 사용자에 의해 종료 요청됨 (Ctrl+C).")
finally:
    print("[시스템] 안전 종료 시퀀스 가동: 모터 토크 해제 중...")
    try:
        # 두 모터에 패킷이 겹치지 않도록 미세한 간격을 두고 토크 해제 명령 전송
        motors.release_torque(PAN_MOTOR_ID)
        time.sleep(0.02)
        motors.release_torque(TILT_MOTOR_ID)
        time.sleep(0.02)
        print("[시스템] 모든 카메라 모터 토크 해제 완료.")
    except Exception as e:
        print(f"[경고] 토크 해제 패킷 전송 실패: {e}")

    motors.close()
    openvr.shutdown()
    print("[시스템] 프로그램이 안전하게 종료되었습니다.")