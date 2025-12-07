from qmc5883l_micropython import create_sensor
import time
from car_control import *
from machine import Pin
import bluetooth
from ble_simple_peripheral import BLESimplePeripheral

# ==== Hàm callback BLE khi nhận lệnh ====
cmd = STOP  # lệnh mặc định
prev_cmd = cmd
led = Pin("LED", Pin.OUT)
stop()
def on_rx(data):
    global cmd
    msg = data.decode().strip().upper()
    print("BLE nhận lệnh:", msg)

    if msg == "GO":
        cmd = GO_AHEAD
    elif msg == "BACK":
        cmd = GO_BACK
    elif msg == "LEFT":
        cmd = TURN_LEFT
    elif msg == "RIGHT":
        cmd = TURN_RIGHT
    elif msg == "STOP":
        cmd = STOP
    else:
        print("Lệnh không hợp lệ:", msg)

#Hàm điều chỉnh góc

def normalizeAngle(angle):
    """Chuyển góc về [-180, 180]"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# ==== Khởi tạo cảm biến ====
time.sleep(3)
sensor = create_sensor()

# ==== Khởi tạo BLE ====
ble = bluetooth.BLE()
sp = BLESimplePeripheral(ble)
sp.on_write(on_rx)

# ==== Hiệu chỉnh la bàn ====
X_min, X_max = 951, 3188
Y_min, Y_max = -3112, -765
Z_min, Z_max = -1845, 373

offset_x = (X_max + X_min) / 2
offset_y = (Y_max + Y_min) / 2
offset_z = (Z_max + Z_min) / 2

scale_x = 2 / (X_max - X_min)
scale_y = 2 / (Y_max - Y_min)
scale_z = 2 / (Z_max - Z_min)

calibration_matrix = [
    [scale_x, 0.0, -offset_x * scale_x],
    [0.0, scale_y, -offset_y * scale_y],
    [0.0, 0.0, 1.0]
]

sensor.calibration = calibration_matrix

print("BLE sẵn sàng. Đang chờ kết nối...")
print("Calibration applied. Bắt đầu đọc dữ liệu...")

heading_now = 0
heading_target = 0
turning = False

# ==== Vòng lặp chính ====
try:
    while True:
        # Đọc dữ liệu cảm biến
        # x, y, z = sensor.get_magnet()
        angle = sensor.get_bearing()
        if angle > 180:
            angle -= 360
        heading_now = angle 
        led.on()
        #print(heading_now)
        # Xử lý khi LỆNH THAY ĐỔI (state transition)
        if cmd != prev_cmd:
            print("Chuyển lệnh:", prev_cmd, "→", cmd)

            if cmd == STOP:
                stop()

            elif cmd == GO_AHEAD:
                go_ahead()

            elif cmd == GO_BACK:
                go_back()

            elif cmd in (TURN_LEFT, TURN_RIGHT):
                # Bắt đầu xoay
                turning = True
                if cmd == TURN_LEFT:
                    heading_target = normalizeAngle(heading_now - 90)
                    turn_left()
                else:
                    heading_target = normalizeAngle(heading_now + 90)
                    turn_right()

            prev_cmd = cmd

    # =================== STATE “TURNING” =====================
        if turning:
            error = normalizeAngle(heading_target - heading_now)
            print("target: ",heading_target)
            if abs(error) <= 3:
                print("Hoàn thành xoay.")
                turning = False
                stop()
                time.sleep(0.5)
                cmd = STOP      
                continue            # vòng lặp sẽ xử lý change-state tiếp theo

    # ==========================================================

    # Debug nhẹ
    # print(f"Góc:{heading_now:.2f}, Cmd:{cmd}, Turning:{turning}")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Kết thúc đọc dữ liệu.")



