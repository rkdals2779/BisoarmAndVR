"""로봇 카메라 영상을 전체화면 창으로 확인하는 스크립트."""

from typing import Final

import cv2

CAMERA_DEVICE_INDEX: Final[int] = 4  # /dev/video4
FRAME_WIDTH: Final[int] = 1280
FRAME_HEIGHT: Final[int] = 960
WINDOW_NAME: Final[str] = 'Robot_Camera'
QUIT_KEY: Final[str] = 'q'


def main() -> None:
	"""카메라를 열고 전체화면 창에 프레임을 스트리밍한다."""
	capture = cv2.VideoCapture(CAMERA_DEVICE_INDEX)
	capture.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
	capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

	# 크기 조절이 가능한 기본 창을 만든 뒤 전체화면 모드로 설정
	cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
	cv2.setWindowProperty(
		WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
	)

	print(f'카메라 스트리밍 중... (종료: {QUIT_KEY})')
	try:
		while True:
			is_frame_read, frame = capture.read()
			if not is_frame_read:
				break

			cv2.imshow(WINDOW_NAME, frame)

			if cv2.waitKey(1) & 0xFF == ord(QUIT_KEY):
				break
	finally:
		capture.release()
		cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
