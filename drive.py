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
        self.MOTOR_SAFE_DELAY = 0.01
        self.EN_DELAY = 0.1
        self.motor_speed = 0.1
        self.init_wall_dist = 0
        
        self.motor_mult = [1.05, 1, 1, 1]
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

        #for pin in self.encoder_pins:
            #GPIO.setup(pin, GPIO.IN)

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

        #for i in range(0, 7, 2):
            #GPIO.add_event_detect(self.encoder_pins[i], GPIO.FALLING, callback=self.read_encoder)

        #thr = threading.Thread(target=self.print_encoder)
        #thr.start()
    
    def calibrate(self, cur_wall_dist):
        diff = abs(cur_wall_dist - self.init_wall_dist)
        if cur_wall_dist > self.init_wall_dist:
            self.motor_mult[0] += diff / 10
            self.motor_mult[1] += diff / 10
            self.motor_mult[2] = 1
            self.motor_mult[3] = 1
        elif cur_wall_dist < self.init_wall_dist:
            self.motor_mult[0] = 1
            self.motor_mult[1] = 1
            self.motor_mult[2] += diff / 10
            self.motor_mult[3] += diff / 10

    def read_encoder(self, channel):
        for i in range(0, 7, 2):
            if channel == self.encoder_pins[i]:
                if GPIO.input(self.encoder_pins[i + 1]):
                    self.encoder_count[i // 2] += 1
                else:
                    self.encoder_count[i // 2] -= 1
    
    def print_encoder(self):
        while True:
            for n in self.encoder_count:
                print(n, end=' ')
            print()
            self.encoder_count = [0, 0, 0, 0]
            time.sleep(self.EN_DELAY)

    def set_speed(self, speed):
        self.motor_speed = speed

    def set_motor(self, robot_direction: str, duration=2.0, auto_stop=True):
        time.sleep(self.MOTOR_SAFE_DELAY)
        for i in range(self.MOTOR_COUNT):
            self.motor[i].throttle = self.speeds[robot_direction][i] * self.motor_speed * self.motor_mult[i]
        if robot_direction != 'stop' and auto_stop and duration > 0:
            t=threading.Timer(duration, self.set_motor, args=['stop'])
            t.start()

    def exit(self):
        self.set_motor('stop')
        self.pca.deinit()