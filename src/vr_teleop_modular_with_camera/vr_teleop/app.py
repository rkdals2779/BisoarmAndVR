"""
텔레옵 애플리케이션 (오케스트레이션)
====================================
config에 정의된 팔(들)을 받아서

    VR 초기화 -> 팔별 로봇/IK 초기화 -> 영점 동기화 -> 실시간 제어 루프

순서로 실행합니다. arms 리스트에 팔이 1개면 단일팔 모드, 2개(왼쪽+오른쪽)면
두 컨트롤러가 각각 다른 로봇을 동시에 움직이는 양팔(bimanual) 모드로
"자동으로" 동작합니다. 팔 개수 외에는 로직 차이가 전혀 없습니다.
"""

import time
from dataclasses import dataclass
from typing import List, Optional

from .camera_head import CameraHeadController
from .config import ArmConfig, TeleopConfig
from .control import ArmTeleopController
from .kinematics import RobotKinematics
from .output import ConsoleStatusDisplay, RobotOutput
from .vr_interface import VRSystem, HMD_DEVICE_INDEX


@dataclass
class _ArmRuntime:
    """팔 하나에 필요한 런타임 객체 묶음 (내부용)"""
    config: ArmConfig
    kin: RobotKinematics
    controller: ArmTeleopController
    output: RobotOutput
    device_idx: Optional[int] = None


class TeleopApp:
    def __init__(self, teleop_config: TeleopConfig):
        if not teleop_config.arms:
            raise ValueError("TeleopConfig.arms가 비어 있습니다. 최소 1개의 ArmConfig가 필요합니다.")
        self.config = teleop_config
        self.vr_system = VRSystem()
        self.arms: List[_ArmRuntime] = []
        self.display = ConsoleStatusDisplay()

        # 팔 개수(1~2)와 무관하게, camera가 설정된 ArmConfig가 있으면
        # (기본: 왼팔) 카메라 헤드도 함께 초기화합니다. 그 팔의 RobotOutput이
        # 이미 열어놓은 시리얼 연결을 공유해서 쓰므로 별도 포트는 열지 않습니다.
        self.camera_head: Optional[CameraHeadController] = None

    # ------------------------------------------------------------
    def setup(self) -> "TeleopApp":
        self.vr_system.connect()

        for arm_cfg in self.config.arms:
            print(f"\n---- [{arm_cfg.name}] ({arm_cfg.controller_role.value} 컨트롤러) 초기화 ----")
            kin = RobotKinematics(
                arm_cfg.ik,
                arm_cfg.wrist.wrist_flex_key,
                arm_cfg.wrist.wrist_roll_key,
                name=arm_cfg.name,
            )
            output = RobotOutput(arm_cfg).connect()
            controller = ArmTeleopController(arm_cfg, kin)
            controller.initialize_from_observation(output.get_observation())

            self.arms.append(_ArmRuntime(config=arm_cfg, kin=kin, controller=controller, output=output))

            if arm_cfg.camera is not None:
                print(f"[{arm_cfg.name}] 카메라 헤드 초기화 중... (공유 포트: {arm_cfg.robot_port})")
                camera_writer = output.get_camera_writer()
                self.camera_head = CameraHeadController(arm_cfg.camera, camera_writer)

        return self

    # ------------------------------------------------------------
    def run(self):
        dt_nominal = self.config.dt_nominal
        last_loop_t = time.perf_counter()
        try:
            # 카운트다운 대기도 try 안에 둬서, 이 시점에 Ctrl+C가 눌려도
            # finally에서 로봇 연결 해제가 항상 실행되도록 합니다.
            self.display.print_calibration_prompt()

            while True:
                loop_start = time.perf_counter()
                dt = max(loop_start - last_loop_t, 1e-3)
                last_loop_t = loop_start

                poses = self.vr_system.get_all_poses()
                any_sent = False

                # 카메라 헤드: get_all_poses()가 이미 받아온 결과에서 HMD
                # 항목만 꺼내 쓰므로 별도의 openvr 호출이 필요 없습니다.
                if self.camera_head is not None:
                    hmd_pose = poses[HMD_DEVICE_INDEX]
                    if hmd_pose.bPoseIsValid:
                        hmd_rot = self.vr_system.extract_rotation_matrix(hmd_pose.mDeviceToAbsoluteTracking)
                        if not self.camera_head.is_calibrated():
                            self.camera_head.calibrate(hmd_rot)
                        else:
                            pan_deg, tilt_deg = self.camera_head.compute_and_send(hmd_rot, loop_start, dt)
                            self.display.set_camera(pan_deg, tilt_deg)
                            any_sent = True

                for runtime in self.arms:
                    runtime.device_idx = self.vr_system.get_controller_index(runtime.config.controller_role)
                    if runtime.device_idx is None:
                        continue

                    pose = poses[runtime.device_idx]
                    if not pose.bPoseIsValid:
                        continue

                    mat = pose.mDeviceToAbsoluteTracking
                    vr_pos = self.vr_system.extract_position(mat)
                    vr_rot = self.vr_system.extract_rotation_matrix(mat)

                    if not runtime.controller.is_calibrated():
                        # 최초로 유효한 pose를 받은 프레임 -> 이번 pose를 영점으로 저장하고
                        # 이번 프레임은 명령을 보내지 않습니다 (델타가 항상 0에서 시작).
                        runtime.controller.calibrate(vr_pos, vr_rot)
                        continue

                    trigger = self.vr_system.get_trigger_value(runtime.device_idx)
                    command = runtime.controller.compute(vr_pos, vr_rot, trigger, loop_start, dt)
                    runtime.output.send(command)
                    self.display.set(runtime.config.name, command)
                    any_sent = True

                if any_sent:
                    self.display.flush()

                elapsed = time.perf_counter() - loop_start
                time.sleep(max(0.0, dt_nominal - elapsed))

        except KeyboardInterrupt:
            print("\n\n[시스템] 안전하게 종료합니다.")
        finally:
            self.shutdown()

    # ------------------------------------------------------------
    def shutdown(self):
        # 카메라 토크 해제는 반드시 로봇 팔 disconnect()보다 먼저 실행합니다.
        # disconnect()가 이 카메라와 공유 중인 시리얼 포트를 닫아버리므로,
        # 그 전에 마지막 패킷(토크 해제)을 먼저 내보내야 합니다.
        if self.camera_head is not None:
            try:
                print("[camera] 카메라 모터 토크 해제 중...")
                self.camera_head.release_torque()
                time.sleep(0.02)
            except Exception as e:
                print(f"[경고] 카메라 헤드 토크 해제 중 오류: {e}")

        for runtime in self.arms:
            try:
                runtime.output.disconnect()
            except Exception as e:
                print(f"[경고] [{runtime.config.name}] 로봇 연결 해제 중 오류: {e}")
        self.vr_system.shutdown()
