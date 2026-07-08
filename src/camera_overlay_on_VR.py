import sys
import time
import cv2
import numpy as np
import openvr
from OpenGL.GL import *
import pygame
from pygame.locals import *

# 1. OpenGL 컨텍스트 생성 (Pygame 활용)
# OpenVR은 OpenGL 텍스처를 요구하므로, 보이지 않는 창을 띄워 OpenGL 환경을 초기화합니다.
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

# 3. 오버레이 위치 및 크기 설정 (HMD 기준 정중앙 고정)
# HmdMatrix34_t를 생성하여 위치를 지정합니다. Z축 음수 방향이 시야 앞쪽입니다.
mat = openvr.HmdMatrix34_t()
mat.m[0][0] = 1.0;
mat.m[0][1] = 0.0;
mat.m[0][2] = 0.0;
mat.m[0][3] = 0.0  # X: 0 (가운데)
mat.m[1][0] = 0.0;
mat.m[1][1] = 1.0;
mat.m[1][2] = 0.0;
mat.m[1][3] = 0.0  # Y: 0 (가운데)
mat.m[2][0] = 0.0;
mat.m[2][1] = 0.0;
mat.m[2][2] = 1.0;
mat.m[2][3] = -1.0  # Z: -1.0 (눈앞 1미터)

# HMD 디바이스 인덱스를 기준으로 위치를 상대적으로 묶어버립니다 (Head-locked)
vr_overlay.setOverlayTransformTrackedDeviceRelative(
    overlay_handle,
    openvr.k_unTrackedDeviceIndex_Hmd,
    mat
)
vr_overlay.setOverlayWidthInMeters(overlay_handle, 1.0)  # 화면 크기 (1미터 폭)
vr_overlay.showOverlay(overlay_handle)

# 4. 카메라 및 OpenGL 텍스처 설정
cap = cv2.VideoCapture(4)  # /dev/video4 연결
if not cap.isOpened():
    print("카메라를 열 수 없습니다. /dev/video4 연결을 확인하세요.")
    sys.exit(1)

# OpenGL 텍스처 ID 생성
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

        # OpenCV(BGR) 이미지를 OpenGL(RGB) 포맷으로 변환
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # OpenGL은 이미지의 원점을 좌측 하단으로 간주하므로 상하 반전이 필요합니다
        frame_rgb = cv2.flip(frame_rgb, 0)
        h, w, _ = frame_rgb.shape

        # OpenGL 텍스처 메모리에 이미지 데이터 업로드
        glBindTexture(GL_TEXTURE_2D, tex_id)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, frame_rgb.tobytes())

        # OpenVR 텍스처 구조체에 OpenGL 텍스처 ID(포인터 역할) 매핑
        ovr_texture = openvr.Texture_t()
        ovr_texture.handle = int(tex_id)  # C++ 포인터 전달을 대신하는 핵심 부분
        ovr_texture.eType = openvr.TextureType_OpenGL
        ovr_texture.eColorSpace = openvr.ColorSpace_Auto

        # 텍스처를 오버레이에 렌더링
        vr_overlay.setOverlayTexture(overlay_handle, ovr_texture)

        # 시스템 부하를 줄이기 위한 약간의 대기시간 (약 60FPS 제한)
        time.sleep(0.016)

except KeyboardInterrupt:
    print("종료 중...")
finally:
    cap.release()
    openvr.shutdown()
    pygame.quit()