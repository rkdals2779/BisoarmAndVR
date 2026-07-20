"""로봇 카메라 영상을 VR HMD 오버레이로 스트리밍하는 스크립트.

카메라 프레임을 OpenGL 텍스처로 올린 뒤 OpenVR 오버레이(SetOverlayTexture)로
매 프레임 갱신합니다.

변경 사항 요약 (v3: SetOverlayRaw -> 다시 SetOverlayTexture로):
	- SetOverlayRaw는 OpenVR 헤더의 VREvent_ImageLoaded/VREvent_ImageFailed
	  주석에서 알 수 있듯, PNG/JPG를 비동기로 읽어들이는 SetOverlayFromFile과
	  같은 계열의 '1회성 이미지 로드' 함수입니다. 매 프레임(16ms 간격)마다
	  새로 호출하면 이전 로드가 끝나기도 전에 다음 요청이 들어가
	  OverlayError_RequestFailed가 발생합니다. 연속 영상에는 맞지 않는
	  API였습니다.
	- 그래서 연속 스트리밍에 맞는 SetOverlayTexture(OpenGL 텍스처)로
	  되돌리되, 처음 InvalidTexture의 유력한 원인이었던 '알파 채널 없는
	  GL_RGB' 포맷을 GL_RGBA8로 바꾸고, 업로드가 끝났음을 보장하는
	  glFlush()를 추가했습니다.
"""

import sys
import time
from typing import Final

import cv2
import openvr
import pygame
from OpenGL.GL import (
	GL_LINEAR,
	GL_RGBA,
	GL_RGBA8,
	GL_TEXTURE_2D,
	GL_TEXTURE_MAG_FILTER,
	GL_TEXTURE_MIN_FILTER,
	GL_UNSIGNED_BYTE,
	glBindTexture,
	glFlush,
	glGenTextures,
	glTexImage2D,
	glTexParameteri,
)
from pygame.locals import DOUBLEBUF, HIDDEN, OPENGL

CAMERA_DEVICE_INDEX: Final[int] = 4  # /dev/video4
OVERLAY_NAME: Final[str] = 'RobotCam'
OVERLAY_WIDTH_M: Final[float] = 1.0
OVERLAY_DISTANCE_M: Final[float] = 1.0  # HMD 눈앞 거리
FRAME_INTERVAL_S: Final[float] = 0.016  # 약 60FPS 제한


def build_hmd_relative_transform() -> 'openvr.HmdMatrix34_t':
	"""HMD 기준 정중앙, 눈앞 OVERLAY_DISTANCE_M 지점의 변환 행렬 생성.

	Returns:
		회전 없이(단위 행렬) -Z 방향으로 OVERLAY_DISTANCE_M 이동한
		3x4 변환 행렬.
	"""
	transform_matrix = openvr.HmdMatrix34_t()
	for row in range(3):
		for col in range(4):
			transform_matrix.m[row][col] = 1.0 if row == col else 0.0
	transform_matrix.m[2][3] = -OVERLAY_DISTANCE_M
	return transform_matrix


def main() -> None:
	"""OpenGL/OpenVR 초기화 후 카메라 -> VR 오버레이 스트리밍 루프 실행."""
	# 1. OpenGL 컨텍스트 생성 (Pygame 활용)
	pygame.init()
	pygame.display.set_mode((100, 100), DOUBLEBUF | OPENGL | HIDDEN)

	# 2. OpenVR 초기화 (오버레이 모드)
	try:
		openvr.init(openvr.VRApplication_Overlay)
	except openvr.OpenVRError as error:
		print(f'OpenVR 초기화 실패: {error}')
		pygame.quit()
		sys.exit(1)

	vr_overlay = openvr.IVROverlay()
	overlay_handle = vr_overlay.createOverlay(
		OVERLAY_NAME, 'Robot Camera View'
	)

	# 3. 오버레이 위치 및 크기 설정 (HMD 기준 정중앙 고정)
	vr_overlay.setOverlayTransformTrackedDeviceRelative(
		overlay_handle,
		openvr.k_unTrackedDeviceIndex_Hmd,
		build_hmd_relative_transform(),
	)
	vr_overlay.setOverlayWidthInMeters(overlay_handle, OVERLAY_WIDTH_M)
	vr_overlay.showOverlay(overlay_handle)

	# 4. 카메라 및 OpenGL 텍스처 설정
	capture = cv2.VideoCapture(CAMERA_DEVICE_INDEX)
	if not capture.isOpened():
		print(
			f'카메라를 열 수 없습니다. '
			f'/dev/video{CAMERA_DEVICE_INDEX} 연결을 확인하세요.'
		)
		openvr.shutdown()
		pygame.quit()
		sys.exit(1)

	texture_id = glGenTextures(1)
	glBindTexture(GL_TEXTURE_2D, texture_id)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

	print('VR 오버레이 스트리밍 시작... (종료하려면 Ctrl+C)')

	# 5. 메인 루프 (카메라 프레임 -> OpenGL 텍스처 -> OpenVR 오버레이)
	try:
		while True:
			is_frame_read, frame = capture.read()
			if not is_frame_read:
				continue

			# OpenCV(BGR) -> OpenGL(RGBA). 알파 채널을 포함해야
			# OverlayError_InvalidTexture를 피할 수 있음.
			frame_rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
			# OpenGL은 이미지 원점을 좌측 하단으로 간주하므로 상하 반전 필요
			frame_rgba = cv2.flip(frame_rgba, 0)
			height, width, _ = frame_rgba.shape

			glBindTexture(GL_TEXTURE_2D, texture_id)
			glTexImage2D(
				GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0,
				GL_RGBA, GL_UNSIGNED_BYTE, frame_rgba.tobytes(),
			)
			# 업로드가 완전히 끝난 뒤에 텍스처 핸들을 컴포지터에 넘기도록 보장
			glFlush()

			ovr_texture = openvr.Texture_t()
			ovr_texture.handle = int(texture_id)
			ovr_texture.eType = openvr.TextureType_OpenGL
			ovr_texture.eColorSpace = openvr.ColorSpace_Auto

			vr_overlay.setOverlayTexture(overlay_handle, ovr_texture)

			# 시스템 부하를 줄이기 위한 약간의 대기시간
			time.sleep(FRAME_INTERVAL_S)

	except KeyboardInterrupt:
		print('종료 중...')
	finally:
		capture.release()
		openvr.shutdown()
		pygame.quit()


if __name__ == '__main__':
	main()
