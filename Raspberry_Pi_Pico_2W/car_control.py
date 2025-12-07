from machine import Pin, PWM
import time

# ======== GPIO CONFIG ==========
BACK_IN1 = Pin(10, Pin.OUT)
BACK_IN2 = Pin(11, Pin.OUT)
BACK_IN3 = Pin(12, Pin.OUT)
BACK_IN4 = Pin(13, Pin.OUT)
BACK_ENA = PWM(Pin(14))
BACK_ENB = PWM(Pin(15))

FRONT_IN1 = Pin(21, Pin.OUT)
FRONT_IN2 = Pin(20, Pin.OUT)
FRONT_IN3 = Pin(19, Pin.OUT)
FRONT_IN4 = Pin(18, Pin.OUT)
FRONT_ENA = PWM(Pin(17))
FRONT_ENB = PWM(Pin(16))

# PWM setup
for pwm in [BACK_ENA, BACK_ENB, FRONT_ENA, FRONT_ENB]:
    pwm.freq(1000)

# ======== TRẠNG THÁI ==========
STOP = 0
GO_AHEAD = 1
TURN_LEFT = 2
TURN_RIGHT = 3
GO_BACK = 4

current_speeds = [0, 0, 0, 0]  # [BACK_ENA, BACK_ENB, FRONT_ENA, FRONT_ENB]

def analogWrite(pwm, duty, index):
    """Ghi PWM và lưu giá trị hiện tại"""
    if duty < 0: duty = 0
    if duty > 255: duty = 255
    pwm.duty_u16(int(duty * 65535 / 255))
    current_speeds[index] = duty


# ================= STOP =================
def stop(step=10, delay_ms=20):
    while any(s > 0 for s in current_speeds):
        for i in range(4):
            current_speeds[i] = max(0, current_speeds[i] - step)
        analogWrite(BACK_ENA, current_speeds[0], 0)
        analogWrite(BACK_ENB, current_speeds[1], 1)
        analogWrite(FRONT_ENA, current_speeds[2], 2)
        analogWrite(FRONT_ENB, current_speeds[3], 3)
        time.sleep_ms(delay_ms)

    # set all pins to 0
    for pin in [
        BACK_IN1, BACK_IN2, BACK_IN3, BACK_IN4,
        FRONT_IN1, FRONT_IN2, FRONT_IN3, FRONT_IN4
    ]:
        pin.value(0)


# ================= GO AHEAD =================
def go_ahead(target_speed=200, step=10, delay_ms=30):
    BACK_IN1.value(1); BACK_IN2.value(0) # banh phai sau BACK_ENA
    BACK_IN3.value(0); BACK_IN4.value(1) # banh trai sau BACK_ENB
    FRONT_IN1.value(0); FRONT_IN2.value(1) # banh trai truoc FRONT_ENA
    FRONT_IN3.value(0); FRONT_IN4.value(1) #banh phai truoc FRONT_ENB

    done = [False, False, False, False]

    while not all(done):
        for i, pwm in enumerate([BACK_ENA, BACK_ENB, FRONT_ENA, FRONT_ENB]):
            if current_speeds[i] < target_speed:
                current_speeds[i] += step
                if current_speeds[i] >= target_speed:
                    current_speeds[i] = target_speed
                    done[i] = True
                analogWrite(pwm, current_speeds[i], i)
        time.sleep_ms(delay_ms)



# ================= GO BACK =================
def go_back(target_speed=150, step=10, delay_ms=30):
    # Cài chiều quay cho bánh khi lùi
    BACK_IN1.value(0); BACK_IN2.value(1)
    BACK_IN3.value(1); BACK_IN4.value(0)
    FRONT_IN1.value(1); FRONT_IN2.value(0)
    FRONT_IN3.value(1); FRONT_IN4.value(0)

    done = [False, False, False, False]

    while not all(done):
        for i, pwm in enumerate([BACK_ENA, BACK_ENB, FRONT_ENA, FRONT_ENB]):
            if current_speeds[i] < target_speed:
                current_speeds[i] += step
                if current_speeds[i] >= target_speed:
                    current_speeds[i] = target_speed
                    done[i] = True

                analogWrite(pwm, current_speeds[i], i)

        time.sleep_ms(delay_ms)



# ================= TURN LEFT =================
def turn_left(speed_left=0, speed_right=220):
    # tiến nhưng lệch tốc độ
    BACK_IN1.value(1); BACK_IN2.value(0) # banh phai sau BACK_ENA
    BACK_IN3.value(0); BACK_IN4.value(1) # banh trai sau BACK_ENB
    FRONT_IN1.value(0); FRONT_IN2.value(1) # banh trai truoc FRONT_ENA
    FRONT_IN3.value(0); FRONT_IN4.value(1) #banh phai truoc FRONT_ENB

    # Không tăng dần – đặt tốc độ ngay
    analogWrite(BACK_ENA, speed_right, 0)
    analogWrite(BACK_ENB, speed_left, 1)
    analogWrite(FRONT_ENA, speed_left, 2)
    analogWrite(FRONT_ENB, speed_right, 3)


# ================= TURN RIGHT =================
def turn_right(speed_right=0, speed_left=220):
    BACK_IN1.value(1); BACK_IN2.value(0)
    BACK_IN3.value(0); BACK_IN4.value(1)
    FRONT_IN1.value(0); FRONT_IN2.value(1)
    FRONT_IN3.value(0); FRONT_IN4.value(1)

    analogWrite(BACK_ENA, speed_right, 0)
    analogWrite(BACK_ENB, speed_left, 1)
    analogWrite(FRONT_ENA, speed_left, 2)
    analogWrite(FRONT_ENB, speed_right, 3)

