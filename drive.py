import time
import busio

from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

class Drive:
    def __init__(self):
        i2c = busio.I2C(SCL, SDA)
        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 100
        self.delay = 1
        self.motor_speed = 0.1
        duty_channels = [0, 3, 6, 9]
        for i in duty_channels:
            pca.channels[i].duty_cycle = 0xFFFF

        # MOTOR NUMBERING
        # 2 1
        # 3 0
        motor_channels = [[1, 2], [4, 5], [7, 8], [10, 11]]
        self.motor = []
        self.MOTOR_COUNT = 4
        self.MOTOR_SAFE_DELAY = 0.01
        for a, b in motor_channels:
            self.motor.append(motor.DCMotor(pca.channels[a], pca.channels[b]))
        for i in range(4):
            self.motor[i].decay_mode = motor.SLOW_DECAY

    def stop(self):
        time.sleep(self.MOTOR_SAFE_DELAY)
        for i in range(self.MOTOR_COUNT):
            self.motor[i].throttle = 0

    def exit(self):
        time.sleep(self.MOTOR_SAFE_DELAY)
        for i in range(self.MOTOR_COUNT):
            self.motor[i].throttle = 0
        pca.deinit()

    def forward(self):
        time.sleep(self.MOTOR_SAFE_DELAY)
        self.motor[0].throttle = -self.motor_speed
        self.motor[1].throttle = -self.motor_speed
        self.motor[2].throttle = self.motor_speed
        self.motor[3].throttle = self.motor_speed

    def backward(self):
        time.sleep(self.MOTOR_SAFE_DELAY)
        self.motor[0].throttle = self.motor_speed
        self.motor[1].throttle = self.motor_speed
        self.motor[2].throttle = -self.motor_speed
        self.motor[3].throttle = -self.motor_speed

    def turn_cw(self):
        time.sleep(self.MOTOR_SAFE_DELAY)
        for i in range(self.MOTOR_COUNT):
            self.motor[i].throttle = self.motor_speed
        # time.sleep(delay)
        # self.stop()

    def turn_ccw(self):
        time.sleep(self.MOTOR_SAFE_DELAY)
        for i in range(self.MOTOR_COUNT):
            self.motor[i].throttle = -self.motor_speed
        # time.sleep(delay)
        # self.stop()

    def left(self):
        time.sleep(self.MOTOR_SAFE_DELAY)
        self.motor[0].throttle = self.motor_speed*2
        self.motor[1].throttle = -self.motor_speed*2
        self.motor[2].throttle = -self.motor_speed*2
        self.motor[3].throttle = self.motor_speed*2

    def right(self):
        time.sleep(self.MOTOR_SAFE_DELAY)
        self.motor[0].throttle = -self.motor_speed*2
        self.motor[1].throttle = self.motor_speed*2
        self.motor[2].throttle = self.motor_speed*2
        self.motor[3].throttle = -self.motor_speed*2


if __name__ == '__main__':
    drive = Drive()
    print("Press f, b, cw, ccw, l, r to go, q : stop, x : exit")
    while True:
        key = input("Where : ")
        if key == 'f':
            drive.forward()
        elif key == 'b':
            drive.backward()
        elif key == 'cw':
            drive.turn_cw()
        elif key == 'ccw':
            drive.turn_ccw()
        elif key == 'q':
            drive.stop()
        elif key == 'x':
            drive.exit()
            break