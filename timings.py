from machine import Pin, PWM, I2C
import time
import math
from bno08x_i2c import *

# Initialize I2C for IMU
I2C1_SDA = Pin(4)
I2C1_SCL = Pin(5)
i2c1 = I2C(0, sda=I2C1_SDA, scl=I2C1_SCL, freq=400000, timeout=200000)
bno = BNO08X_I2C(i2c1, debug=False)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
leda = Pin(1, Pin.OUT) 
ledb = Pin(6, Pin.OUT) 
leda.value(1)  
ledb.value(1)
class Encoder:
    def __init__(self, pin_x, pin_y, reverse=False, scale=1):
        self.reverse = reverse
        self.scale = scale
        self.forward = True
        self.pin_x = Pin(pin_x, Pin.IN, Pin.PULL_UP)
        self.pin_y = Pin(pin_y, Pin.IN, Pin.PULL_UP)
        self._pos = 0
        try:
            self.x_interrupt = self.pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.x_callback, hard=True)
            self.y_interrupt = self.pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.y_callback, hard=True)
        except TypeError:
            self.x_interrupt = self.pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.x_callback)
            self.y_interrupt = self.pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.y_callback)

    def x_callback(self, pin):
        self.forward = self.pin_x.value() ^ self.pin_y.value() ^ self.reverse
        self._pos += 1 if self.forward else -1

    def y_callback(self, pin):
        self.forward = self.pin_x.value() ^ self.pin_y.value() ^ self.reverse ^ 1
        self._pos += 1 if self.forward else -1

    def position(self, value=None):
        if value is not None:
            self._pos = round(value / self.scale)
        return self._pos * self.scale

    def reset(self):
        self._pos = 0

    def value(self, value=None):
        if value is not None:
            self._pos = value
        return self._pos

# Motor A Encoder pins using GPIO 15 and 14
encoder_a = Encoder(14, 15)
encoder_b= Encoder(11,10)


# Motor control pins
ENA = Pin(16, Pin.OUT)
IN1 = Pin(17, Pin.OUT)
IN2 = Pin(18, Pin.OUT)
IN3 = Pin(20, Pin.OUT)
IN4 = Pin(19, Pin.OUT)
ENB = Pin(21, Pin.OUT)
# Set up PWM for motors
pwm_a = PWM(ENA)
pwm_b = PWM(ENB)
pwm_a.init(freq=10000, duty_u16=300000) # type: ignore
pwm_b.init(freq=10000, duty_u16=300000) # type: ignore

# PID Parameters
Kp_straight = 0.05
Ki_straight = 0.0
Kd_straight = 0
Kp_distance = 0.01
Ki_distance = 0.0
Kd_distance = 0.0
Kp_turn = 0.01
Ki_turn = 0.00000
Kd_turn = 0.1
deadband_distance = 10
deadband_turn = 0.07

# Push button on GPIO 22
button = Pin(22, Pin.IN, Pin.PULL_UP)
turn_count=0
straight_count=0
turn_time = 3.5 # time for one turn (in seconds)
straight_time = 0.8  # time for one straight at 50% speed (in seconds)
min_speed=0.32
max_turn_speed=0.32


def normalize_angle(angle):
    """Normalize an angle to the range [0, 360)."""
    return (angle+360) % 360

def set_motor_speed_a(speed):
    motor_speed_1 = int(abs(speed) * 65535)
    if speed > 0:
        IN1.value(1)
        IN2.value(0)
    elif speed < 0:
        IN1.value(0)
        IN2.value(1)
    if speed==0:  # Brake mode
        IN1.value(1)
        IN2.value(1)
    pwm_a.duty_u16(motor_speed_1)

def set_motor_speed_b(speed):
    motor_speed_2 = int(abs(speed) * 65535)
    if speed > 0:
        IN3.value(1)
        IN4.value(0)
    elif speed < 0:
        IN3.value(0)
        IN4.value(1)
    pwm_b.duty_u16(motor_speed_2)

def calculate_speed(x):
    global motor_speed
    motor_speed=0
    time_elapsed=time.time()-start_time
    time_at_destination=(turn_count*turn_time)+((straight_count+x)*time_per_straight)
    time_to_destination= time_at_destination-time_elapsed
    time_per_subsegment=time_to_destination/x
    motor_speed = 0.50 * (straight_time / time_per_subsegment)
    if motor_speed > 1:
        motor_speed = 1
    elif motor_speed < 0.32:
        motor_speed = 0.32
    return motor_speed


def forward(segments):
    global straight_count
    straight_count+=segments
    yaw = normalize_angle(bno.euler[2])
    straight_error = target_yaw - yaw
    error_sum_straight = 0
    last_error_straight = 0
    encoder_a = Encoder(14, 15)
    encoder_b= Encoder(11,10)
    encoder_b.reset()
    encoder_a.reset()
    total_distance=segments*25
    wheel_diameter=6
    encoder_resolution=1440
    wheel_circumference=math.pi*wheel_diameter
    encoder_distance=(total_distance/wheel_circumference)*encoder_resolution/abs((math.cos(straight_error)))
    traveled_distance=(encoder_a.position()+(-1*encoder_b.position()))/2
    distance_error=encoder_distance-traveled_distance
    while abs(distance_error)>deadband_distance:
        yaw = normalize_angle(bno.euler[2])
        straight_error = target_yaw - yaw
        traveled_distance=(encoder_a.position()+(-1*encoder_b.position()))/2
        distance_error=encoder_distance-traveled_distance
        segments_left=segments-distance_error*wheel_circumference/encoder_resolution
        calculate_speed(segments_left)
        # Normalize the turn error
        if straight_error > 180:
            straight_error -= 360
        elif straight_error < -180:
            straight_error += 360

        # Turn PID calculations
        P_straight = straight_error * Kp_straight
        I_straight = error_sum_straight * Ki_straight
        D_straight = (straight_error - last_error_straight) * Kd_straight
        correction_straight = P_straight + I_straight + D_straight
        if abs(distance_error)<1440:
        #Distance PID Calculations:
            P_distance = distance_error * Kp_distance
        else:
            P_distance=0
        motor_b_speed=-2.13414*motor_speed**3+3.74527*motor_speed**2-1.21031*motor_speed+0.429783
        # Calculate motor speeds based on correction
        speed_a = motor_speed + correction_straight + P_distance
        speed_b = motor_b_speed - correction_straight + P_distance

        # Apply minimum limits to avoid stalling
        if abs(speed_a) > 0 and abs(speed_a) < min_speed:
            speed_a = min_speed if speed_a > 0 else -min_speed
        if abs(speed_b) > 0 and abs(speed_b) < min_speed:
            speed_b = min_speed if speed_b > 0 else -min_speed

        # Optionally clamp speeds to max limits
        speed_a = max(min(speed_a, 1), -1)
        speed_b = max(min(speed_b, 1), -1)

        # Send speeds to motors
        set_motor_speed_a(1)
        set_motor_speed_b(1)

        error_sum_straight += straight_error
        last_error_straight = straight_error
        
        #print(f"yaw: {yaw}, distance traveled: {traveled_distance}, target distance: {encoder_distance}, Speed A: {speed_a}, Speed B: {speed_b},Desired Speed: {motor_speed}")
    
    set_motor_speed_a(-0.05)
    set_motor_speed_b(-0.05)
    global initial_yaw
    initial_yaw=target_yaw
    time.sleep(0.2)
# Main loop to check for button press and execute commands
while True:
    if button.value() == 0:  # Button pressed (assuming active low)
        print("Button pressed, starting sequence...")
        # Example sequence of function calls
        # Example usage
        global start_time
        #print(start_time)
        leda.value(0)  
        ledb.value(0)
        target_time = 0.8
        turn_num = 0
        straight_num = 1
        total_turn_time = turn_time * turn_num
        remaining_time = target_time - total_turn_time
        time_per_straight = remaining_time / straight_num
        target_yaw = normalize_angle(bno.euler[2])
        set_motor_speed_a(0.05)
        set_motor_speed_b(0.05)
        start_time=time.time_ns()
        forward(1)
        print(time.time_ns()-start_time)
        #turn_left()
        # turn_left()
        # forward(3)
        # turn_right()
        # forward(1)
        # backward(1)
        # turn_left()
        # forward(2)
        leda.value(1)  
        ledb.value(1)
        # Debounce delay
        # time.sleep(1)
    set_motor_speed_a(0.05)
    set_motor_speed_b(0.05)
    # Short delay to prevent button bouncing
    time.sleep(0.1)