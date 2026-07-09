# VR 텔레옵 컨트롤러 (모듈화 버전)

원본 단일 파일 스크립트(`Vr_teleop_xyz_pitch_roll_right_controller.py`)를 역할별로
모듈화하고, 왼쪽/오른쪽 컨트롤러를 선택하거나 두 컨트롤러로 로봇 2대를 동시에
움직이는 **양손(bimanual) 텔레옵**을 지원하도록 확장한 버전입니다.

제어 로직(필터 파라미터, IK 방식, 손목 직접 매핑, 임계감쇠 궤적 컨트롤러 등)은
원본과 **완전히 동일**합니다. 바뀐 것은 구조와, 좌/우/양팔을 고를 수 있게 된 부분뿐입니다.

## 폴더 구조

```
vr_teleop_project/
├── main.py                  # 실행 진입점 (CLI로 left/right/dual 모드 선택)
├── README.md
└── vr_teleop/
    ├── config.py             # ★ 모든 설정값 (여길 가장 많이 보게 됩니다)
    ├── geometry.py            # pose 행렬 <-> 위치/회전 변환 (순수 수학, 하드웨어 무관)
    ├── filters.py             # One Euro Filter (적응형 스무딩)
    ├── trajectory.py           # 임계감쇠 관절 궤적 컨트롤러
    ├── kinematics.py           # ikpy 기반 IK 체인 관리
    ├── vr_interface.py          # OpenVR SDK 래퍼 (입력부)
    ├── control.py              # 팔 1개의 텔레옵 계산 파이프라인 (제어부)
    ├── camera_head.py           # 카메라 pan/tilt 헤드 제어 (로봇 팔과 시리얼 포트 공유)
    ├── output.py               # 로봇 명령 전송 + 콘솔 상태 출력 (출력부)
    └── app.py                 # 전체 오케스트레이션 (단일팔/양팔 공용)
```

### 데이터 흐름 (하나의 팔 기준)

```
vr_interface.py (VR pose/trigger 읽기)
        │
        ▼
control.py — ArmTeleopController.compute()
    · geometry.py의 pitch/roll 추출
    · filters.py의 OneEuroFilter (위치용/회전용 각각)
    · kinematics.py의 RobotKinematics (IK, 관절 한계 clip)
    · trajectory.py의 JointTrajectoryController
        │
        ▼
output.py — RobotOutput.send() (실제 서보로 전송) + ConsoleStatusDisplay (상태줄 출력)
```

`app.py`의 `TeleopApp`이 이 전체를 묶어서 반복 실행합니다. **팔이 1개면 단일팔,
2개(왼쪽+오른쪽)면 양팔 모드**이며, 그 외의 로직은 완전히 동일한 코드가 돕니다.

## 설치

```bash
pip install numpy scipy openvr ikpy pyserial
# + lerobot (SO-101 팔로워 드라이버, 사내/원본 설치 방식대로)
```

## 설정하기 — `vr_teleop/config.py`

로봇 포트, URDF 경로, 스케일, 필터 강도, 손목 부호(sign) 등 **모든 튜닝값은
이 파일 하나에만** 있습니다. 특히 아래 두 프리셋을 실제 환경에 맞게 고치세요.

```python
RIGHT_ARM_CONFIG = ArmConfig(
    name="right_arm",
    controller_role=ControllerRole.RIGHT,
    robot_id="skm_right_follower",
    robot_port="/dev/ttyACM1",
    home_pos=np.array([0.2, 0.0, 0.1]),
    ik=IKConfig(urdf_path="/path/to/so101_new_calib.urdf"),
)

LEFT_ARM_CONFIG = ArmConfig(
    name="left_arm",
    controller_role=ControllerRole.LEFT,
    robot_id="skm_left_follower",
    robot_port="/dev/ttyACM0",
    home_pos=np.array([0.2, 0.0, 0.1]),
    ik=IKConfig(urdf_path="/path/to/so101_new_calib.urdf"),
)
```

양팔 모드에서 왼쪽 로봇이 오른쪽과 거울 대칭으로 장착되어 특정 축이 반대로
움직이면, `ArmConfig.axis_signs`(위치, 기본 `(1,1,1)`)나
`WristMappingConfig.pitch_sign` / `roll_sign`(손목, 기본 각각 `-1.0`/`1.0`)을
`-1.0`으로 뒤집어서 조정하세요. 둘 다 팔마다 독립적으로 설정할 수 있습니다.

## 실행하기

```bash
# 오른쪽 컨트롤러로 오른팔 로봇 1대만 제어 (원본 스크립트와 동일한 동작)
python main.py --mode right

# 왼쪽 컨트롤러로 왼팔 로봇 1대만 제어
python main.py --mode left

# 양쪽 컨트롤러로 로봇 2대를 동시에 제어 (양손 텔레옵)
python main.py --mode dual

# 제어 주기를 바꾸고 싶을 때
python main.py --mode dual --hz 60
```

실행하면 원본과 동일하게: IK 체인/관절 진단 출력 → 3초 카운트다운(컨트롤러 영점
조절) → 실시간 추종이 시작됩니다. `Ctrl+C`로 언제든 안전하게 종료되며(카운트다운
중이어도) 모든 로봇의 연결이 정리됩니다.

## 카메라 pan/tilt 헤드 (HMD 추종)

왼팔 로봇과 카메라 pan/tilt 모터(기본 id 7/8)가 **같은 물리적 시리얼 버스**
(`/dev/ttyACM0` 하나에 데이지체인으로 연결)에 물려 있는 구성을 위한 기능입니다.

**왜 원래 두 스크립트를 같이 돌리면 오류가 났는가:** 로봇 팔 텔레옵(lerobot)과
카메라 트래커가 각자 독립적으로 `serial.Serial("/dev/ttyACM0")`를 열었기
때문입니다. 같은 하프듀플렉스 버스에 두 개의 연결이 동시에 패킷을 쓰면
서로 겹쳐 써서 체크섬이 깨지고, 응답을 읽을 때도 어느 연결이 그 바이트를
가져갈지 보장되지 않아 양쪽 다 통신 오류/타임아웃이 발생합니다.

**해결 방법:** `camera_head.py`는 별도의 포트를 열지 않습니다. 대신
`find_shared_serial()`이 왼팔 `RobotOutput`(lerobot의 `SO101Follower`)이
이미 열어놓은 `serial.Serial` 객체를 객체 그래프에서 찾아내 그대로
재사용합니다(속성 이름이 아니라 타입으로 찾으므로 lerobot 내부 구현이
바뀌어도 비교적 안전합니다). 그 결과 버스에는 항상 하나의 쓰기 주체만
존재하게 되어 충돌이 사라집니다.

동작 방식:
- `LEFT_ARM_CONFIG.camera`에 `CameraHeadConfig`가 붙어 있으면, `left`/`dual`
  모드에서 자동으로 카메라 추종이 함께 시작됩니다 (`right` 단독 모드에서는
  왼팔 연결 자체가 없으므로 카메라도 동작하지 않습니다).
- 컨트롤러 캘리브레이션과 같은 3초 카운트다운 시점에 HMD 정면 방향도 함께
  영점으로 잡힙니다.
- 카메라 모터가 없거나 다른 포트에 있다면 `--no-camera` 플래그로 끌 수
  있습니다: `python main.py --mode dual --no-camera`

튜닝 포인트 (`vr_teleop/config.py`의 `CameraHeadConfig`):
| 하고 싶은 것 | 필드 |
|---|---|
| 모터 id 변경 | `pan_motor_id` / `tilt_motor_id` |
| 좌우/상하 반대로 움직임 | `yaw_sign` / `pitch_sign`을 -1.0으로 |
| 가동 범위(선 꼬임 방지) | `pan_min_deg`~`pan_max_deg`, `tilt_min_deg`~`tilt_max_deg` |
| 반응성/부드러움 | `rot_filter`(One Euro), `trajectory`(속도/가속도 제한) |

## 다른 설정 조합이 필요하다면

`main.py`를 거치지 않고 직접 조합할 수도 있습니다. 예를 들어 왼쪽 컨트롤러로
"오른팔" 로봇을 조작하고 싶다면:

```python
from vr_teleop.app import TeleopApp
from vr_teleop.config import TeleopConfig, RIGHT_ARM_CONFIG, ControllerRole
import dataclasses

cfg = dataclasses.replace(RIGHT_ARM_CONFIG, controller_role=ControllerRole.LEFT)
app = TeleopApp(TeleopConfig(control_hz=50.0, arms=[cfg]))
app.setup()
app.run()
```

로봇을 3대 이상 쓰고 싶다면(예: 발 페달로 3번째 팔 트리거 등) `ArmConfig`를
더 만들어 `arms` 리스트에 추가하기만 하면 됩니다 — `app.py`는 팔 개수에
대한 가정을 두지 않습니다.

## 수정 포인트 요약

| 하고 싶은 것 | 건드릴 파일 |
|---|---|
| 로봇 포트/URDF/스케일/필터/손목 부호 튜닝 | `config.py` |
| VR 트리거 축 번호, 컨트롤러 인식 방식 변경 | `vr_interface.py` |
| IK 방식/관절 한계 처리 변경 | `kinematics.py` |
| 필터링 알고리즘 자체를 교체 | `filters.py` |
| 궤적 스무딩(속도/가속도 제한) 방식 변경 | `trajectory.py` |
| 텔레옵 계산 파이프라인(단계 순서, 매핑 공식) 변경 | `control.py` |
| 카메라 pan/tilt 모터 id/범위/부호, 공유 시리얼 탐색 로직 변경 | `camera_head.py` (설정값은 `config.py`) |
| 다른 로봇 SDK로 교체, 콘솔 출력 형식 변경 | `output.py` |
| 실행 순서/여러 팔 동시 실행 로직 변경 | `app.py` |
