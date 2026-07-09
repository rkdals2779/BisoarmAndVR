import sys
import time

import cv2
import numpy as np
import openvr
from OpenGL.GL import *
import pygame
from pygame.locals import *

# ------------------------------------------------------------------------
# 변경 사항 요약 (v3: SetOverlayRaw -> 다시 SetOverlayTexture로)
#   - SetOverlayRaw는 OpenVR 헤더의 VREvent_ImageLoaded/VREvent_ImageFailed
#     주석에서 알 수 있듯, PNG/JPG를 비동기로 읽어들이는 SetOverlayFromFile과
#     같은 계열의 "1회성 이미지 로드" 함수입니다. 매 프레임(16ms 간격)마다
#     새로 호출하면 이전 로드가 끝나기도 전에 다음 요청이 들어가
#     OverlayError_RequestFailed가 발생합니다. 연속 영상에는 맞지 않는 API였습니다.
#   - 그래서 연속 스트리밍에 맞는 SetOverlayTexture(OpenGL 텍스처)로 되돌리되,
#     처음 InvalidTexture의 유력한 원인이었던 "알파 채널 없는 GL_RGB" 포맷을
#     GL_RGBA8로 바꾸고, 업로드가 끝났음을 보장하는 glFlush()를 추가했습니다.
# ------------------------------------------------------------------------

# 1. OpenGL 컨텍스트 생성 (Pygame 활용)
pygame.init()
pygame.display.set_mode((100, 100), DOUBLEBUF | OPENGL | HIDDEN)

# 2. OpenVR 초기화 (오버레이 모드)
try:
    openvr.init(openvr.VRApplication_Overlay)
except openvr.OpenVRError as e:
    print(f"OpenVR 초기화 실패: {e}")
    sys.exit(1)

vr_overlay = openvr.IVROverlay()
overlay_handle = vr_overlay.createOverlay("RobotCam", "Robot Camera View")

# 3. 오버레이 위치 및 크기 설정 (HMD 기준 정중앙 고정, 눈앞 1미터)
mat = openvr.HmdMatrix34_t()
mat.m[0][0] = 1.0; mat.m[0][1] = 0.0; mat.m[0][2] = 0.0; mat.m[0][3] = 0.0
mat.m[1][0] = 0.0; mat.m[1][1] = 1.0; mat.m[1][2] = 0.0; mat.m[1][3] = 0.0
mat.m[2][0] = 0.0; mat.m[2][1] = 0.0; mat.m[2][2] = 1.0; mat.m[2][3] = -1.0

vr_overlay.setOverlayTransformTrackedDeviceRelative(
    overlay_handle,
    openvr.k_unTrackedDeviceIndex_Hmd,
    mat
)
vr_overlay.setOverlayWidthInMeters(overlay_handle, 1.0)
vr_overlay.showOverlay(overlay_handle)

# 4. 카메라 및 OpenGL 텍스처 설정
cap = cv2.VideoCapture(4)  # /dev/video4
if not cap.isOpened():
    print("카메라를 열 수 없습니다. /dev/video4 연결을 확인하세요.")
    sys.exit(1)

tex_id = glGenTextures(1)
glBindTexture(GL_TEXTURE_2D, tex_id)
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

print("VR 오버레이 스트리밍 시작... (종료하려면 Ctrl+C)")

# 5. 메인 루프 (카메라 프레임 -> OpenGL 텍스처 -> OpenVR 오버레이)
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # OpenCV(BGR) -> OpenGL(RGBA). 알파 채널을 포함해야 OverlayError_InvalidTexture를 피할 수 있음
        frame_rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        # OpenGL은 이미지 원점을 좌측 하단으로 간주하므로 상하 반전 필요
        frame_rgba = cv2.flip(frame_rgba, 0)
        h, w, _ = frame_rgba.shape

        glBindTexture(GL_TEXTURE_2D, tex_id)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, frame_rgba.tobytes())
        # 업로드가 완전히 끝난 뒤에 텍스처 핸들을 컴포지터에 넘기도록 보장
        glFlush()

        ovr_texture = openvr.Texture_t()
        ovr_texture.handle = int(tex_id)
        ovr_texture.eType = openvr.TextureType_OpenGL
        ovr_texture.eColorSpace = openvr.ColorSpace_Auto

        vr_overlay.setOverlayTexture(overlay_handle, ovr_texture)

        # 시스템 부하를 줄이기 위한 약간의 대기시간 (약 60FPS 제한)
        time.sleep(0.016)

except KeyboardInterrupt:
    print("종료 중...")
finally:
    cap.release()
    openvr.shutdown()
    pygame.quit()