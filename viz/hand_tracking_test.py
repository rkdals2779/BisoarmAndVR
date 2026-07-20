#!/usr/bin/env python3
"""
check_hand_tracking.py
=============================================================================
matplotlib 없이 콘솔에서만, SteamVR로부터 손 스켈레톤(bone) 데이터가
실제로 들어오고 있는지 진단하는 스크립트.

확인하는 것
-----------
1. SteamVR에 연결된 트래킹 장치 목록 (Quest 손이 장치로 잡히는지)
2. 스켈레톤 액션이 실제 장치에 "바인딩"되어 있는지 (getActionOrigins)
3. 스켈레톤 액션이 "활성(active)" 상태인지 (bActive)
4. 활성 상태라면 손목/검지 끝 좌표가 프레임마다 실제로 바뀌는지

실행
----
    python3 check_hand_tracking.py

Ctrl+C로 종료.
"""

import json
import sys
import tempfile
import time
from pathlib import Path
from typing import Final

try:
	import openvr
except ImportError:
	sys.exit('[오류] pip install openvr 로 openvr 패키지를 먼저 설치하세요.')


ACTION_SET_PATH: Final[str] = '/actions/handviz'
ACTION_LEFT: Final[str] = '/actions/handviz/in/lefthand_anim'
ACTION_RIGHT: Final[str] = '/actions/handviz/in/righthand_anim'

# 손목(1)과 검지 끝 보조 뼈(27번, Aux_IndexFinger)만 샘플로 출력한다.
SAMPLE_BONES: Final[dict[str, int]] = {'Wrist': 1, 'IndexTip(Aux)': 27}


def build_action_manifest(target_dir: Path) -> Path:
	target_dir.mkdir(parents=True, exist_ok=True)
	manifest = {
		'action_sets': [{'name': ACTION_SET_PATH, 'usage': 'leftright'}],
		'actions': [
			{
				'name': ACTION_LEFT,
				'type': 'skeleton',
				'skeleton': '/skeleton/hand/left',
			},
			{
				'name': ACTION_RIGHT,
				'type': 'skeleton',
				'skeleton': '/skeleton/hand/right',
			},
		],
		'default_bindings': [],
		'localization': [
			{
				'language_tag': 'en_US',
				ACTION_SET_PATH: 'Hand Bone Visualizer',
				ACTION_LEFT: 'Left Hand Skeleton',
				ACTION_RIGHT: 'Right Hand Skeleton',
			}
		],
	}
	# 기존 vr_skeletal_tracker_viz.py 와 동일한 경로를 사용해
	# SteamVR이 같은 앱/바인딩으로 인식할 가능성을 높인다.
	manifest_path = target_dir / 'handviz_actions.json'
	with open(manifest_path, 'w', encoding='utf-8') as manifest_file:
		json.dump(manifest, manifest_file, indent=2, ensure_ascii=False)
	return manifest_path


def describe_tracked_devices(vr_system: 'openvr.IVRSystem') -> None:
	print('\n[1] 현재 SteamVR에 연결된 트래킹 장치')
	print('-' * 60)
	class_names = {
		openvr.TrackedDeviceClass_Invalid: 'Invalid',
		openvr.TrackedDeviceClass_HMD: 'HMD',
		openvr.TrackedDeviceClass_Controller: 'Controller',
		openvr.TrackedDeviceClass_GenericTracker: 'GenericTracker',
		openvr.TrackedDeviceClass_TrackingReference: 'TrackingReference',
	}
	has_found_device = False
	for i in range(openvr.k_unMaxTrackedDeviceCount):
		try:
			device_class = vr_system.getTrackedDeviceClass(i)
		except Exception:
			continue
		if device_class == openvr.TrackedDeviceClass_Invalid:
			continue
		has_found_device = True
		is_connected = vr_system.isTrackedDeviceConnected(i)
		try:
			model = vr_system.getStringTrackedDeviceProperty(
				i, openvr.Prop_RenderModelName_String
			)
		except Exception:
			model = '?'
		print(
			f'  장치 #{i:>2}  '
			f'class={class_names.get(device_class, device_class):<18} '
			f'connected={is_connected}  model={model}'
		)
	if not has_found_device:
		print('  (연결된 장치가 하나도 없습니다 — SteamVR/ALVR 연결부터 확인하세요)')
	print('-' * 60)


def describe_action_origins(
	vr_input: 'openvr.IVRInput',
	action_set_handle: int,
	action_handle: int,
	label: str,
) -> None:
	"""이 액션이 실제로 어떤 입력 소스(장치)에 바인딩되어 있는지 확인."""
	try:
		origins = vr_input.getActionOrigins(
			action_set_handle, action_handle, 16
		)
	except Exception as error:
		print(f'    [{label}] getActionOrigins 호출 실패: {error}')
		return

	valid_origins = [
		origin for origin in origins
		if origin != openvr.k_ulInvalidInputValueHandle
	]
	if not valid_origins:
		print(f'    [{label}] ⚠ 바인딩된 입력 소스가 없습니다 (binding 문제일 가능성 높음)')
		return

	for origin in valid_origins:
		name = None
		try:
			name = vr_input.getOriginLocalizedName(
				origin, openvr.k_unMaxActionOriginCount
			)
		except Exception:
			pass
		print(f'    [{label}] 바인딩된 입력 소스: {name if name else origin}')


def main() -> None:
	try:
		openvr.init(openvr.VRApplication_Background)
	except Exception as error:
		sys.exit(
			'[오류] SteamVR에 연결하지 못했습니다. SteamVR/ALVR 연결 상태를 확인하세요.\n'
			f'원본 오류: {error}'
		)

	try:
		vr_system = openvr.VRSystem()
		vr_input = openvr.VRInput()

		manifest_dir = Path(tempfile.gettempdir()) / 'steamvr_hand_visualizer'
		manifest_path = build_action_manifest(manifest_dir)
		vr_input.setActionManifestPath(str(manifest_path))

		action_set_handle = vr_input.getActionSetHandle(ACTION_SET_PATH)
		action_handles = {
			'left': vr_input.getActionHandle(ACTION_LEFT),
			'right': vr_input.getActionHandle(ACTION_RIGHT),
		}

		describe_tracked_devices(vr_system)

		print('[2] 액션 바인딩 상태 확인')
		print('-' * 60)
		for hand in ('left', 'right'):
			describe_action_origins(
				vr_input, action_set_handle, action_handles[hand], hand
			)
		print('-' * 60)

		print(
			'\n[3] 실시간 스켈레톤 데이터 확인 (0.5초 간격, Ctrl+C로 종료)\n'
			'    -> 손을 카메라 앞에서 움직였을 때 좌표 값이 계속 바뀌면 정상입니다.\n'
		)

		last_print_time = 0.0
		while True:
			action_set = openvr.VRActiveActionSet_t()
			action_set.ulActionSet = action_set_handle
			action_set.ulRestrictedToDevice = openvr.k_ulInvalidInputValueHandle
			action_set.nPriority = 0
			vr_input.updateActionState([action_set])

			now = time.time()
			if now - last_print_time >= 0.5:
				last_print_time = now
				timestamp = time.strftime('%H:%M:%S')
				print(f'--- {timestamp} ' + '-' * 40)

				for hand in ('left', 'right'):
					action_handle = action_handles[hand]
					try:
						action_data = vr_input.getSkeletalActionData(
							action_handle
						)
					except Exception as error:
						print(f'  [{hand}] getSkeletalActionData 오류: {error}')
						continue

					is_active = getattr(action_data, 'bActive', False)
					bone_count = getattr(action_data, 'boneCount', 0)

					if not is_active:
						print(f'  [{hand}] bActive=False  (바인딩 또는 트래킹 유실)')
						continue

					try:
						bones = vr_input.getSkeletalBoneData(
							action_handle,
							openvr.VRSkeletalTransformSpace_Model,
							openvr.VRSkeletalMotionRange_WithoutController,
							bone_count or 31,
						)
					except Exception as error:
						print(f'  [{hand}] getSkeletalBoneData 오류: {error}')
						continue

					sample_strs = []
					for name, bone_index in SAMPLE_BONES.items():
						if bone_index < len(bones):
							position = bones[bone_index].position.v
							sample_strs.append(
								f'{name}=({position[0]:+.3f}, '
								f'{position[1]:+.3f}, '
								f'{position[2]:+.3f})'
							)
					print(
						f'  [{hand}] bActive=True boneCount={bone_count}  '
						+ '  '.join(sample_strs)
					)

			time.sleep(1.0 / 90.0)

	except KeyboardInterrupt:
		print('\n종료합니다.')
	finally:
		openvr.shutdown()


if __name__ == '__main__':
	main()