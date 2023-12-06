import time
import busio
import Jetson.GPIO as GPIO
from typing import List
import threading

from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

class Drive:
    def __init__(self):
        self.imu = []
        self.MOTOR_SAFE_DELAY = 0.01
        self.EN_DELAY = 0.1
        self.motor_speed = 0.2
        self.init_wall_dist = 0
        self.cur_action = 'stop'
        self.prev_action = 'stop'
        
        self.motor_mult = [1, 1, 1, 1]
        self.speeds = {
            'stop': [0, 0, 0, 0],
            'forward': [-1, -1, 1, 1],
            'backward': [1, 1, -1, -1],
            'turn_cw': [1, 1, 1, 1],
            'turn_ccw': [-1, -1, -1, -1],
            'left': [1.5, -1.5, -1.5, 1.5],
            'right': [-1.5, 1.5, 1.5, -1.5]
        }

        self.encoder_pins = [
            'SPI1_MOSI',
            'SPI1_MISO',
            'SPI1_SCK',
            'CAM_AF_EN',
            'GPIO_PZ0',
            'GPIO_PE6',
            'DAP4_FS',
            'SPI2_MOSI'
        ]
        self.encoder_count = [0, 0, 0, 0]

        GPIO.cleanup()
        GPIO.setmode(GPIO.TEGRA_SOC)

        for pin in self.encoder_pins:
            GPIO.setup(pin, GPIO.IN)

        # MOTOR INIT
        # MOTOR NUMBERING
        # 0 3
        # 1 2
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c, address=0x40)
        self.pca.frequency = 100
        duty_channels = [0, 3, 6, 9]
        for i in duty_channels:
            self.pca.channels[i].duty_cycle = 0xFFFF
        motor_channels = [[1, 2], [4, 5], [7, 8], [10, 11]]
        self.motor: List[motor.DCMotor] = []
        self.MOTOR_COUNT = 4
        for a, b in motor_channels:
            self.motor.append(motor.DCMotor(self.pca.channels[a], self.pca.channels[b]))
        for i in range(4):
            self.motor[i].decay_mode = motor.FAST_DECAY

        for i in range(0, 7, 2):
            GPIO.add_event_detect(self.encoder_pins[i], GPIO.RISING, callback=self.read_encoder)

        #thr = threading.Thread(target=self.print_encoder)
        #thr.start()
    
    def calibrate(self, cur_wall_dist):
        if self.cur_action != 'forward':     #forward only for now
            return
        if self.init_wall_dist == 0 or self.prev_action != self.cur_action:
            self.init_wall_dist = cur_wall_dist
        diff = abs(cur_wall_dist - self.init_wall_dist)
        if diff > 0.5:
            self.motor_mult[0] = 1
            self.motor_mult[1] = 1
            self.motor_mult[2] = 1
            self.motor_mult[3] = 1
            self.init_wall_dist = cur_wall_dist
            self.set_motor(self.cur_action, 0, False)
        if diff > 0.03:
            if cur_wall_dist > self.init_wall_dist:
                print("adjusting to right")
                self.motor_mult[0] = 0.7
                self.motor_mult[1] = 0.7
                self.motor_mult[2] = 1
                self.motor_mult[3] = 1
            elif cur_wall_dist < self.init_wall_dist:
                print("adjusting to left")
                self.motor_mult[0] = 1
                self.motor_mult[1] = 1
                self.motor_mult[2] = 0.7
                self.motor_mult[3] = 0.7
            self.set_motor(self.cur_action, 0, False)
        self.prev_action = self.cur_action

    def read_encoder(self, channel):
        for i in range(0, 7, 2):
            if channel == self.encoder_pins[i]:
                if GPIO.input(self.encoder_pins[i + 1]):
                    self.encoder_count[i // 2] += 1
                else:
                    self.encoder_count[i // 2] -= 1
        print(self.encoder_count)
    
    def print_encoder(self):
        while True:
            for n in self.encoder_count:
                print(n, end=' ')
            print()
            time.sleep(self.EN_DELAY)

    def set_speed(self, speed):
        self.motor_speed = speed

    def stop_motor_reverse_direction(self):
        if self.cur_action == 'forward':
            self.set_motor('backward', 0.1, True, False)
        elif self.cur_action == 'backward':
            self.set_motor('forward', 0.1, True, False)
        elif self.cur_action == 'left':
            self.set_motor('right', 0.1, True, False)
        elif self.cur_action == 'right':
            self.set_motor('left', 0.1, True, False)

    def set_motor(self, robot_direction: str, duration=2.0, auto_stop=True, reverse=True):
        # print(robot_direction + ' ' + str(duration) + ' ' + str(auto_stop) + ' ' + str(reverse))
        time.sleep(self.MOTOR_SAFE_DELAY)
        for i in range(self.MOTOR_COUNT):
            speed = self.speeds[robot_direction][i] * self.motor_speed * self.motor_mult[i]
            if speed > 1.0 or speed < -1.0: # debug for incorrect value
                print(speed)
            self.motor[i].throttle = speed

        if robot_direction == 'stop' and reverse:
            # reset motor multiplier
            # print("reversing for stop")
            # print(self.cur_action)
            self.init_wall_dist = 0
            self.motor_mult = [1, 1, 1, 1]
            self.stop_motor_reverse_direction()

        # auto stop
        if robot_direction != 'stop' and auto_stop and duration > 0:
            t=threading.Timer(duration, self.set_motor, args=['stop', 0, False, False])
            t.start()

        self.cur_action = robot_direction

    def turn_degrees(self, degrees: float, direction: str):
        prev = self.imu[2]
        if degrees == 90:
            rotation_val = 0.67
        elif degrees == 180:
            rotation_val = 1.03
        else:
            rotation_val = 0
        rot_sum = 0.0
        # imu value is between -1 and 1
        # 180 degrees = 1
        
        prev_speed = self.motor_speed
        self.set_speed(0.15)
        if direction == 'cw':
            self.set_motor('turn_cw', 0, False)
        elif direction == 'ccw':
            self.set_motor('turn_ccw', 0, False)
        
        time.sleep(0.06)
        while rot_sum < rotation_val:
            rot_sum += abs(self.imu[2] - prev)
            prev = self.imu[2]
            time.sleep(0.01)
        self.set_motor('stop', 0, False)
        self.set_speed(prev_speed)
    
    def turn_degrees_encoder(self, degrees: float, direction: str):
        current_encoder = self.encoder_count[2]
        if direction == 'cw':
            self.set_motor('turn_cw', 0, False)
        elif direction == 'ccw':
            self.set_motor('turn_ccw', 0, False)
        thr = threading.Thread(target=self.check_encoder, args=[degrees, current_encoder])
        thr.start()
    
    def check_encoder(self, degrees: float, current_encoder: int):
        rot_val = 800 * degrees / 360
        while True:
            if abs(self.encoder_count[3] - current_encoder) >= rot_val:
                self.set_motor('stop', 0, False)
                break
            time.sleep(0.01) # put this delay or encoder value will lock here forever

    def exit(self):
        self.set_motor('stop')
        self.pca.deinit()