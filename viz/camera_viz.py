import cv2

cap = cv2.VideoCapture(4)  # /dev/video4인 경우 4
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1600)

print("카메라 스트리밍 중... (종료: q)")
while True:
    ret, frame = cap.read()
    if not ret: break

    cv2.imshow("Robot_Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()