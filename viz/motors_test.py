import serial
import time


def scan_feetech_motors(port="/dev/ttyACM0", baudrate=1000000, max_id=15):
    print(f"🔍 포트 {port}에서 1부터 {max_id}까지 모터 ID를 스캔합니다... (통신속도: {baudrate})")

    try:
        # 시리얼 포트 열기 (Feetech 모터 기본 통신속도는 보통 1,000,000)
        ser = serial.Serial(port, baudrate, timeout=0.05)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        print("포트 권한이 없다면 'sudo chmod 666 /dev/ttyACM1' 명령어를 먼저 실행해주세요.")
        return

    found_ids = []

    for motor_id in range(1, max_id + 1):
        # Feetech PING 패킷 생성: [FF, FF, ID, Length, Instruction, Checksum]
        length = 0x02
        instruction = 0x01  # PING 명령어
        # 체크섬 계산: ~(ID + Length + Instruction)의 하위 8비트
        checksum = (~(motor_id + length + instruction)) & 0xFF

        packet = bytearray([0xFF, 0xFF, motor_id, length, instruction, checksum])

        ser.reset_input_buffer()
        ser.write(packet)

        # 응답 대기 (핑 응답은 6바이트)
        response = ser.read(6)

        # 응답 패킷 헤더(FF FF) 및 ID 일치 확인
        if len(response) >= 6 and response[0] == 0xFF and response[1] == 0xFF:
            recv_id = response[2]
            error_code = response[4]

            if recv_id == motor_id:
                print(f"✅ 모터 발견! ID: {motor_id:2d} (상태/에러 코드: {error_code})")
                found_ids.append(motor_id)

    ser.close()

    print("\n--- 📊 스캔 결과 ---")
    if found_ids:
        print(f"연결이 확인된 모터 ID 목록: {found_ids}")
        print(f"총 {len(found_ids)}개의 모터가 응답했습니다.")
    else:
        print("❌ 연결된 모터를 찾을 수 없습니다.")
        print("체크포인트: 1. 모터 전원 공급 상태, 2. 데이터 케이블 연결 상태, 3. 통신 속도(Baudrate)")


if __name__ == "__main__":
    # 포트 이름과 최대 스캔할 ID 갯수를 지정합니다.
    scan_feetech_motors(port="/dev/ttyACM1", baudrate=1000000, max_id=15)