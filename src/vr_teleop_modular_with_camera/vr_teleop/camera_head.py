"""
카메라 pan/tilt 헤드 제어
=========================
HMD(헤드셋) 방향을 따라가는 카메라 pan/tilt 모터(Feetech, 기본 id 7/8)를
제어합니다.

--------------------------------------------------------------------
왜 원래 코드가 로봇 팔 텔레옵과 같이 돌리면 오류가 났는가
--------------------------------------------------------------------
카메라 모터는 로봇 팔(왼팔)과 "물리적으로 같은 시리얼 버스"
(/dev/ttyACM0 하나에 여러 Feetech 서보가 데이지체인으로 연결된 구조)에
달려 있습니다. 원래의 단일 파일 카메라 스크립트는 이 사실과 무관하게
자기 혼자 `serial.Serial(CAMERA_PORT)`로 포트를 새로 열었습니다.

로봇 팔 쪽(lerobot의 SO101Follower)도 같은 포트를 이미 열어서 쓰고
있으므로, 결과적으로 같은 하프듀플렉스 RS-485/TTL 버스 위에 두 개의
독립된 Serial 연결이 동시에 패킷을 씁니다. 두 연결이 타이밍을 전혀
모르는 채로 겹쳐 쓰면:
    - 한쪽이 쓰는 도중에 다른 쪽이 끼어들어 바이트가 섞이고
    - 응답을 읽을 때도 어느 쪽 연결이 그 바이트를 가져갈지 보장이 안 되어
    - 체크섬이 깨지거나 응답이 아예 안 와서 통신 오류/타임아웃이 납니다.

그래서 이 모듈은 절대로 serial.Serial을 새로 열지 않습니다. 대신
find_shared_serial()로 로봇 팔 쪽(RobotOutput.robot)이 이미 열어놓은
연결 객체를 찾아서 "같은 객체"를 그대로 재사용합니다 - 버스의 유일한
쓰기 주체를 하나로 유지하는 것이 근본적인 해결책입니다.
"""

from typing import Optional

import numpy as np
import serial

from .config import CameraHeadConfig
from .filters import OneEuroFilter
from .geometry import extract_yaw_pitch
from .trajectory import JointTrajectoryController


# ============================================================
# 공유 시리얼 연결 탐색
# ============================================================
def find_shared_serial(obj, _seen: Optional[set] = None, _depth: int = 0, _max_depth: int = 6) -> Optional[serial.Serial]:
    """
    obj(예: lerobot의 SO101Follower 인스턴스)의 속성들을 재귀적으로 뒤져서
    이미 열려 있는 serial.Serial 인스턴스를 찾아 반환합니다.

    lerobot 내부 구현(속성 이름이 bus/port_handler/ser 등 무엇인지, 몇 단계
    깊이 있는지)은 버전에 따라 달라질 수 있습니다. 그래서 속성 "이름"이
    아니라 "타입"으로 찾습니다 - 우리가 진짜로 필요한 건 실제 열려 있는
    시리얼 연결 객체 그 자체이고, 그걸 찾아 재사용하는 게 목적이므로
    이 방식이 내부 구조 변경에 가장 안전합니다.

    찾지 못하면 None을 반환합니다 (호출부에서 절대 새 포트를 열어
    fallback하지 않도록, 명확한 에러로 이어지게 하세요).
    """
    if _seen is None:
        _seen = set()
    if id(obj) in _seen or _depth > _max_depth:
        return None
    _seen.add(id(obj))

    if isinstance(obj, serial.Serial):
        return obj if obj.is_open else None

    obj_dict = getattr(obj, "__dict__", None)
    if not obj_dict:
        return None

    for value in obj_dict.values():
        found = find_shared_serial(value, _seen, _depth + 1, _max_depth)
        if found is not None:
            return found
    return None


# ============================================================
# 저수준 Feetech 패킷 작성 (원본 단일 파일 스크립트와 동일한 프로토콜)
# ============================================================
class SharedFeetechWriter:
    """
    이미 열려 있는 serial.Serial 연결 위에서 WRITE_DATA(0x03) 명령으로
    Feetech 서보에 직접 패킷을 씁니다.

    포트를 새로 열거나 닫지 않습니다 - 연결의 소유권(열고/닫는 책임)은
    항상 로봇 팔 쪽 RobotOutput(lerobot)에 있고, 이 클래스는 그 연결을
    빌려서 쓰기만 합니다.
    """

    def __init__(self, shared_serial: serial.Serial):
        self.ser = shared_serial

    def write_position(self, motor_id: int, pos_deg: float):
        # 0~360도를 0~4095로 변환
        pos_counts = int(pos_deg * 4096 / 360.0)
        pos_counts = max(0, min(4095, pos_counts))
        pos_l = pos_counts & 0xFF
        pos_h = (pos_counts >> 8) & 0xFF

        length = 9
        cmd = 0x03
        addr = 0x2A  # Target Position Register
        checksum = ~(motor_id + length + cmd + addr + pos_l + pos_h + 0 + 0 + 0 + 0) & 0xFF

        packet = bytearray([
            0xFF, 0xFF, motor_id, length, cmd, addr,
            pos_l, pos_h, 0x00, 0x00, 0x00, 0x00, checksum
        ])
        self.ser.write(packet)

    def release_torque(self, motor_id: int):
        """토크를 해제해서 손으로 굴릴 수 있는 Free 상태로 만듭니다."""
        length = 4
        cmd = 0x03
        addr = 0x28  # Torque Enable Register
        val = 0x00
        checksum = ~(motor_id + length + cmd + addr + val) & 0xFF

        packet = bytearray([0xFF, 0xFF, motor_id, length, cmd, addr, val, checksum])
        self.ser.write(packet)


# ============================================================
# 카메라 헤드 텔레옵 계산 (HMD yaw/pitch -> pan/tilt 목표각)
# ============================================================
class CameraHeadController:
    """
    HMD 방향을 따라가는 pan/tilt 계산 상태 머신.
    control.py의 ArmTeleopController와 같은 패턴(calibrate 1회 -> 매 프레임
    compute)을 따르므로 app.py의 메인 루프에 그대로 끼워 넣을 수 있습니다.
    """

    def __init__(self, config: CameraHeadConfig, writer: SharedFeetechWriter):
        self.config = config
        self.writer = writer

        self.rot_filter = OneEuroFilter(
            min_cutoff=config.rot_filter.min_cutoff,
            beta=config.rot_filter.beta,
            d_cutoff=config.rot_filter.d_cutoff,
        )
        self.trajectory = JointTrajectoryController(
            kp=config.trajectory.kp,
            max_vel_deg_s=config.trajectory.max_vel_deg_s,
            max_acc_deg_s2=config.trajectory.max_acc_deg_s2,
        )
        self.trajectory.reset([config.pan_home_deg, config.tilt_home_deg])

        self._home_rot: Optional[np.ndarray] = None

    def is_calibrated(self) -> bool:
        return self._home_rot is not None

    def calibrate(self, hmd_rot: np.ndarray):
        """이번 HMD 방향을 영점으로 저장하고, 모터를 홈 포지션으로 보냅니다."""
        self._home_rot = hmd_rot.copy()
        self.writer.write_position(self.config.pan_motor_id, self.config.pan_home_deg)
        self.writer.write_position(self.config.tilt_motor_id, self.config.tilt_home_deg)

    def compute_and_send(self, hmd_rot: np.ndarray, t: float, dt: float):
        """calibrate()가 이미 호출된 상태에서 매 프레임 호출합니다.
        반환값은 (pan_deg, tilt_deg) - 콘솔 상태 표시용."""
        cfg = self.config

        rel_rot = self._home_rot.T @ hmd_rot
        yaw_raw, pitch_raw = extract_yaw_pitch(rel_rot)
        yaw_filt, pitch_filt = self.rot_filter.filter(np.array([yaw_raw, pitch_raw]), t)

        pan_target = cfg.pan_home_deg + cfg.yaw_sign * np.degrees(yaw_filt) * cfg.yaw_scale
        tilt_target = cfg.tilt_home_deg + cfg.pitch_sign * np.degrees(pitch_filt) * cfg.pitch_scale
        pan_target = float(np.clip(pan_target, cfg.pan_min_deg, cfg.pan_max_deg))
        tilt_target = float(np.clip(tilt_target, cfg.tilt_min_deg, cfg.tilt_max_deg))

        smoothed = self.trajectory.update([pan_target, tilt_target], dt)
        self.writer.write_position(cfg.pan_motor_id, float(smoothed[0]))
        self.writer.write_position(cfg.tilt_motor_id, float(smoothed[1]))
        return float(smoothed[0]), float(smoothed[1])

    def release_torque(self):
        self.writer.release_torque(self.config.pan_motor_id)
        self.writer.release_torque(self.config.tilt_motor_id)
