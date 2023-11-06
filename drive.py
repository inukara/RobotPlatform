import time
import busio
from typing import List

from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

class Drive:
    def __init__(self):
        self.MOTOR_SAFE_DELAY = 0.01
        self.motor_speed = 0.1

        self.STOP = 0
        self.FORWARD = 1
        self.BACKWARD = 2
        self.TURN_CW = 3
        self.TURN_CCW = 4
        self.LEFT = 5
        self.RIGHT = 6
        
        self.speeds = [
            [0, 0, 0, 0], # STOP
            [-self.motor_speed, -self.motor_speed, self.motor_speed, self.motor_speed], # FORWARD
            [self.motor_speed, self.motor_speed, -self.motor_speed, -self.motor_speed], # BACKWARD
            [self.motor_speed, self.motor_speed, self.motor_speed, self.motor_speed], # TURN_CW
            [-self.motor_speed, -self.motor_speed, -self.motor_speed, -self.motor_speed], # TURN_CCW
            [self.motor_speed*2, -self.motor_speed*2, -self.motor_speed*2, self.motor_speed*2], # LEFT
            [-self.motor_speed*2, self.motor_speed*2, self.motor_speed*2, -self.motor_speed*2] # RIGHT
        ]

        # MOTOR INIT
        # MOTOR NUMBERING
        # 2 1
        # 3 0
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
            self.motor[i].decay_mode = motor.SLOW_DECAY

    def set_motor(self, robot_direction, speed_multiplier=1):
        time.sleep(self.MOTOR_SAFE_DELAY)
        for i in range(self.MOTOR_COUNT):
            self.motor[i].throttle = self.speeds[robot_direction][i] * speed_multiplier

    def exit(self):
        self.set_motor(self.STOP)
        self.pca.deinit()