import serial
import time

FORWARD_SPEED_MM_S = 5
BACKWARD_SPEED_MM_S = -5
TURN_SPEED = 5
STEP_DURATION_SEC = 2
SEND_INTERVAL_SEC = 0.05

def send_motion(ser, x_speed=0, y_speed=0, z_speed=0):
    """
    x_speed: 前後速度 (mm/s)
    y_speed: 左右速度 (mm/s)
    z_speed: 轉向速度 (依控制器定義)
    """
    # 1. 構建數據幀 (依照手冊格式)
    frame = bytearray(11)
    frame[0] = 0x7B           # 幀頭
    frame[1] = 0x00           # 預留位
    frame[2] = 0x00           # 預留位
    
    # 將速度轉為 16 位元整數 (High/Low byte)
    # x_speed 是 mm/s
    x_hex = int(x_speed).to_bytes(2, byteorder='big', signed=True)
    frame[3] = x_hex[1]       # X軸速度低位 (LSB)
    frame[4] = x_hex[0]       # X軸速度高位 (MSB)
    
    y_hex = int(y_speed).to_bytes(2, byteorder='big', signed=True)
    frame[5] = y_hex[1]       # Y軸速度低位
    frame[6] = y_hex[0]       # Y軸速度高位

    z_hex = int(z_speed).to_bytes(2, byteorder='big', signed=True)
    frame[7] = z_hex[1]       # Z軸速度低位 (轉向)
    frame[8] = z_hex[0]       # Z軸速度高位
    
    # 2. 計算 BCC 校驗位 (前 9 位異或運算)
    bcc = 0
    for i in range(9):
        bcc ^= frame[i]
    frame[9] = bcc            # 校驗位
    
    frame[10] = 0x7D          # 幀尾

    # 3. 發送數據
    ser.write(frame)
    print(f"發送指令: {frame.hex(' ').upper()}")


def run_step(ser, label, x_speed=0, y_speed=0, z_speed=0, duration=STEP_DURATION_SEC):
    print(f"開始動作: {label} ({duration} 秒)")
    start_time = time.time()
    while time.time() - start_time < duration:
        send_motion(ser, x_speed, y_speed, z_speed)
        time.sleep(SEND_INTERVAL_SEC)

    print(f"結束動作: {label}，停止小車")
    send_motion(ser, 0, 0, 0)
    time.sleep(0.5)

# --- 主程式 ---
try:
    # 初始化串口
    # Windows 範例: 'COM5', Linux 範例: '/dev/ttyUSB0'
    ser = serial.Serial('COM5', 115200, timeout=1)
    print("串口已連線，等待 10 秒安全保護期...")
    time.sleep(3) # 手冊規定的電機保護期

    print("開始測試：前、後、左、右")
    run_step(ser, "前進", x_speed=FORWARD_SPEED_MM_S)
    run_step(ser, "後退", x_speed=BACKWARD_SPEED_MM_S)

    # 阿克曼底盤常見做法：前進同時給轉向角速度測左/右
    run_step(ser, "左轉", x_speed=FORWARD_SPEED_MM_S, z_speed=TURN_SPEED)
    run_step(ser, "右轉", x_speed=FORWARD_SPEED_MM_S, z_speed=-TURN_SPEED)

    print("測試結束：停止小車")
    send_motion(ser, 0, 0, 0)
    ser.close()

except Exception as e:
    print(f"錯誤: {e}")