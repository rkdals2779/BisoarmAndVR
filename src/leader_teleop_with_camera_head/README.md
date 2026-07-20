# leader_teleop_with_camera_head

텔레오퍼레이션(`lerobot-teleoperate`)과 데이터 수집(`lerobot-record`)에
**카메라 헤드(pan/tilt, 기본 모터 id 7/8) 제어**와 **팔 조종 소스
선택(리더암 / VR 컨트롤러)**을 더한 실행 스크립트 모음.

| 진입점 | 역할 |
|--------|------|
| `main.py` | 텔레오퍼레이션 + 카메라 헤드 (lerobot-teleoperate 대응) |
| `record_main.py` | 데이터 수집 + 카메라 헤드 (lerobot-record 대응) |

- lerobot 소스는 수정하지 않는다. 실행 환경에 설치된 lerobot의
  config/제어 루프 구성 요소를 그대로 가져다 쓴다
  (**0.5.x / 0.6.x 모두 지원** — 시각화 API 차이는 내부에서 호환 처리).
- record의 에피소드 루프(`record_loop`)는 lerobot 것을 수정 없이
  사용한다 — `get_action()` 호출 시 카메라 헤드도 함께 갱신하는 래퍼
  텔레오퍼레이터(`teleop_factory.TeleopWithCameraHead`)를 끼우는 방식.
- 카메라 헤드 모터는 **팔로워 팔과 같은 시리얼 버스**(기본: 왼팔)에
  데이지체인으로 연결돼 있다고 가정한다. 새 시리얼 포트를 열지 않고
  팔로워가 이미 열어놓은 lerobot `FeetechMotorsBus`를 공유해서 raw id
  7/8에 저수준 쓰기를 한다 → 포트 이중 오픈으로 인한 패킷 충돌 없음.
- 버스 쓰기는 전부 메인 루프(단일 스레드)에서만 일어난다. 키보드/VR
  입력 스레드는 목표값 변수만 갱신한다.

## 실행

VRroboseasy(pyenv) 환경에는 lerobot 0.5.1과 pynput/openvr/scipy/rerun이
모두 설치돼 있어 네 가지 모드를 추가 설치 없이 쓸 수 있다.

```bash
pyenv activate VRroboseasy  # 또는 사용 중인 lerobot 환경
cd src/leader_teleop_with_camera_head

python main.py \
  --robot.type=bi_so_follower \
  --robot.id=bi_follower \
  --robot.left_arm_config.port=/dev/so101_follower_left \
  --robot.left_arm_config.cameras='{"top": {"type": "opencv", "index_or_path": "/dev/cam_top", "width": 640, "height": 480, "fps": 30}, "wrist_left": {"type": "opencv", "index_or_path": "/dev/cam_wrist_left", "width": 640, "height": 480, "fps": 30}}' \
  --robot.right_arm_config.port=/dev/so101_follower_right \
  --robot.right_arm_config.cameras='{"wrist_right": {"type": "opencv", "index_or_path": "/dev/cam_wrist_right", "width": 640, "height": 480, "fps": 30}}' \
  --teleop.type=bi_so_leader \
  --teleop.id=bi_leader \
  --teleop.left_arm_config.port=/dev/so101_leader_left \
  --teleop.right_arm_config.port=/dev/so101_leader_right \
  --display_data=true \
  --camera_head.mode=keyboard
```

`--camera_head.*`를 제외한 모든 인자는 `lerobot-teleoperate`와 동일하다.
종료는 Ctrl+C — 카메라 헤드 토크 해제 → 시각화 종료 → 리더/팔로워
disconnect 순서로 정리된다.

## 데이터 수집 (record_main.py)

`lerobot-record`와 동일한 인자(`--dataset.*` 포함)에 `--camera_head.*`만
더하면 된다. 에피소드 제어 키도 동일: `→` 조기 종료, `←` 재녹화,
`ESC` 수집 중지.

```bash
python record_main.py \
  --robot.type=bi_so_follower \
  --robot.id=bi_follower \
  --robot.left_arm_config.port=/dev/so101_follower_left \
  --robot.left_arm_config.cameras='{"top": {"type": "opencv", "index_or_path": "/dev/cam_top", "width": 640, "height": 480, "fps": 30}, "wrist_left": {"type": "opencv", "index_or_path": "/dev/cam_wrist_left", "width": 640, "height": 480, "fps": 30}}' \
  --robot.right_arm_config.port=/dev/so101_follower_right \
  --robot.right_arm_config.cameras='{"wrist_right": {"type": "opencv", "index_or_path": "/dev/cam_wrist_right", "width": 640, "height": 480, "fps": 30}}' \
  --teleop.type=bi_so_leader \
  --teleop.id=bi_leader \
  --teleop.left_arm_config.port=/dev/so101_leader_left \
  --teleop.right_arm_config.port=/dev/so101_leader_right \
  --dataset.repo_id=roboseasy/pick_place \
  --dataset.single_task='물건을 집어 상자에 넣는다' \
  --dataset.num_episodes=10 \
  --dataset.push_to_hub=false \
  --display_data=true \
  --camera_head.mode=vr
```

주의: record 중에는 방향키가 에피소드 제어에 쓰이므로, 카메라 헤드
keyboard 모드는 자동으로 **wasd 배치**(a/d=pan, w/s=tilt, h=홈)로
전환된다.

**카메라 헤드 데이터 기록**: `--camera_head.mode`가 `none`이 아니면
데이터셋에 헤드 상태가 함께 저장된다 —
action에는 명령값, observation에는 버스에서 읽은 실측값이
`camera_head_pan.pos` / `camera_head_tilt.pos` 항목으로 들어간다
(실측 읽기 실패 시 마지막 명령값으로 fallback). `mode=none`이면 헤드
항목 없이 lerobot-record와 완전히 같은 스키마가 되므로, 같은
데이터셋에 이어서 수집(resume)할 때는 헤드 모드 사용 여부를 처음과
동일하게 맞춰야 한다.

## 팔 조종 소스 선택 (`--teleop.type=...`)

| 타입 | 조종 방식 | 필요 하드웨어 |
|------|----------|--------------|
| `bi_so_leader` | 리더암을 손으로 움직여 팔로워암 추종 | 리더암 2대 (시리얼 포트) |
| `bi_vr_leader` | VR 컨트롤러 위치 델타 → IK, 트리거 → 그리퍼 | VR 컨트롤러 + SteamVR(ALVR) |

`bi_vr_leader`는 기존 `vr_teleop` 패키지의 계산 파이프라인(One Euro
Filter → 위치 IK → 손목 직접 매핑 → 임계감쇠 궤적)을 재사용하며,
포트 인자가 필요 없다. 주요 옵션: `--teleop.mode=dual|left|right`,
`--teleop.scale_factor=0.8`, `--teleop.urdf_path=...`(기본: 레포 URDF).
최초로 유효한 컨트롤러 pose가 영점이 되고, 이후 델타만큼 움직인다.
카메라 헤드 vr 모드와 함께 쓰면 SteamVR 세션 하나를 자동으로 공유한다.

## 카메라 헤드 모드 (`--camera_head.mode=...`)

| 모드 | 동작 | 추가 의존성 |
|------|------|------------|
| `none` (기본) | 헤드 제어 안 함 | - |
| `fixed` | 시작 시 고정 각도로 이동 후 유지 | - |
| `keyboard` | 방향키 좌/우=pan, 상/하=tilt, `Home`=홈 복귀 (record에선 wasd) | pynput (VRroboseasy에 설치됨) |
| `vr` | SteamVR HMD 방향(yaw/pitch)을 pan/tilt로 추종 | openvr·scipy (설치됨) + SteamVR(ALVR) 실행 |

- keyboard 모드는 pynput 전역 리스너를 쓰므로 X11 데스크톱 세션에서
  실행해야 한다. 이동 방향이 반대면
  `camera_head/keyboard_controller.py`의 `PAN_KEY_SIGN`/`TILT_KEY_SIGN`
  부호를 뒤집는다.
- vr 모드는 최초로 유효한 HMD pose를 받은 방향이 영점이 된다. 방향이
  반대면 `--camera_head.vr_yaw_sign=-1.0` 류의 부호 옵션으로 뒤집는다.
  필터·궤적 계산은 `../vr_teleop_modular_with_camera/vr_teleop`의
  OneEuroFilter / JointTrajectoryController를 재사용한다.

## 주요 옵션 (기본값)

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--camera_head.bus_arm` | `left` | 헤드 모터가 물린 팔로워 팔 (`left`/`right`) |
| `--camera_head.pan_motor_id` | `7` | pan 모터 id |
| `--camera_head.tilt_motor_id` | `8` | tilt 모터 id |
| `--camera_head.pan_home_deg` | `180.0` | pan 홈 각도 |
| `--camera_head.tilt_home_deg` | `220.0` | tilt 홈 각도 |
| `--camera_head.pan_min_deg` / `pan_max_deg` | `90` / `270` | pan 리밋 (deg) |
| `--camera_head.tilt_min_deg` / `tilt_max_deg` | `110` / `250` | tilt 리밋 (deg) |
| `--camera_head.fixed_pan_deg` / `fixed_tilt_deg` | 홈 각도 | fixed 모드 목표각 |
| `--camera_head.keyboard_speed_deg_s` | `60.0` | keyboard 모드 이동 속도 |
| `--camera_head.vr_yaw_scale` / `vr_pitch_scale` | `1.0` / `-1.0` | HMD→헤드 배율 |
| `--camera_head.vr_yaw_sign` / `vr_pitch_sign` | `-1.0` / `1.0` | 방향 부호 |

모든 목표각은 전송 직전 리밋으로 clamp되고, 종료 시(정상/Ctrl+C/에러
공통) id 7/8 토크가 버스가 닫히기 전에 해제된다.

## 파일 구성

```
main.py                          # 텔레옵 진입점 (lerobot 텔레옵 루프 + 헤드)
record_main.py                   # 데이터 수집 진입점 (lerobot record 대응)
teleop_factory.py                # 팔 소스 팩토리 + 헤드 갱신 래퍼 텔레옵
camera_head/
	config.py                    # CameraHeadMode / CameraHeadConfig
	driver.py                    # 공유 Feetech 버스 저수준 드라이버
	controllers.py               # 베이스/fixed 컨트롤러 + 팩토리
	keyboard_controller.py       # keyboard 모드 (pynput, arrows/wasd)
	vr_controller.py             # vr 모드 (openvr/scipy, vr_teleop 재사용)
vr_arm/
	config.py                    # BiVrLeaderConfig (--teleop.type 등록)
	teleop.py                    # BiVrLeader (VR 컨트롤러 팔 텔레옵)
```
