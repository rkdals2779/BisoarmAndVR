import cv2

cap = cv2.VideoCapture(4)  # /dev/video4인 경우 4
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

# 1. 창의 이름을 변수로 지정 (오타 방지)
window_name = 'Robot_Camera'

# 2. 크기 조절이 가능한 기본 창 생성
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

# 3. 해당 창을 전체화면(FULLSCREEN) 모드로 설정
cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

print('카메라 스트리밍 중... (종료: q)')
while True:
	ret, frame = cap.read()
	if not ret: break

	# 4. 전체화면으로 설정된 창에 프레임 출력
	cv2.imshow(window_name, frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()