import time
import busio
import Jetson.GPIO as GPIO
from typing import List
import asyncio

from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

imu = []

class Drive:
    def __init__(self):
        self.MOTOR_SAFE_DELAY = 0.01
        self.EN_DELAY = 0.1
        self.motor_speed = 0.2
        self.init_wall_dist = 0
        self.cur_action = 'stop'
        self.prev_action = 'stop'
        
        self.motor_default_mult = [1, 1, 1, 1]
        self.motor_mult = self.motor_default_mult
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

    # temp code for calibrating, unused
    async def calibrate(self, cur_wall_dist):
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
            await self.set_motor(self.cur_action, 0, False)
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
            await self.set_motor(self.cur_action, 0, False)
        self.prev_action = self.cur_action

    # read motor's encoders
    def read_encoder(self, channel):
        for i in range(0, 7, 2):
            if channel == self.encoder_pins[i]:
                if GPIO.input(self.encoder_pins[i + 1]):
                    self.encoder_count[i // 2] += 1
                else:
                    self.encoder_count[i // 2] -= 1
        #print(self.encoder_count)

    # set robot's global speed
    def set_speed(self, speed):
        self.motor_speed = speed

    # motor reversing for slipping when stopping
    async def stop_motor_reverse_direction(self):
        if self.cur_action == 'forward':
            await self.set_motor('backward', 0.1, True, False)
        elif self.cur_action == 'backward':
            await self.set_motor('forward', 0.1, True, False)
        elif self.cur_action == 'left':
            await self.set_motor('right', 0.1, True, False)
        elif self.cur_action == 'right':
            await self.set_motor('left', 0.1, True, False)

    # set motor to move robot to selected direction
    async def set_motor(self, robot_direction: str, duration=2.0, auto_stop=True, reverse=True):
        # print(robot_direction + ' ' + str(duration) + ' ' + str(auto_stop) + ' ' + str(reverse))
        await asyncio.sleep(self.MOTOR_SAFE_DELAY)
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
            self.motor_mult = self.motor_default_mult
            await self.stop_motor_reverse_direction()

        # auto stop
        if robot_direction != 'stop' and auto_stop and duration > 0:
            await asyncio.sleep(duration)
            await self.set_motor('stop', 0, False, False)

        self.cur_action = robot_direction

    # rotate robot by degrees
    async def turn_degrees(self, degrees, direction: str, second_run = False):
        try:
            print(f"turning {degrees} degrees {direction}")
            global imu
            prev = imu[2]
            
            '''
            if degrees == 180:
                if abs(imu[2]) < 0.8:
                    rotation_val = 1
                else:
                    rotation_val = 0.6
            '''
            if degrees == 90:
                if second_run:
                    rotation_val = 0.3
                elif abs(imu[2]) < 0.1 or (abs(imu[2]) > 0.6 and abs(imu[2]) < 0.7):
                    rotation_val = 0.68
                else:
                    rotation_val = 0.28
            else:
                print("invalid degrees")
                return
            
            #print(rotation_val)
            init_imu = imu[2]
            rot_sum = 0.0
            # imu value is between -1 and 1
            # 180 degrees = 1
            
            prev_speed = self.motor_speed
            self.set_speed(0.15)
            if direction == 'cw':
                await self.set_motor('turn_cw', 0, False, False)
            elif direction == 'ccw':
                await self.set_motor('turn_ccw', 0, False, False)
            
            await asyncio.sleep(0.06)
            while rot_sum < rotation_val:
                print(rot_sum)
                '''
                if (abs(imu[2]) > 0.98 and abs(init_imu) < 0.95) or (abs(imu[2]) < 0.02 and abs(init_imu) < 0.05) or ((abs(imu[2]) > 0.68 and abs(imu[2]) < 0.72) and not (abs(init_imu) > 0.65 and abs(init_imu) < 0.75)):
                    break
                '''
                rot_sum += abs(imu[2] - prev)
                prev = imu[2]
                await asyncio.sleep(0.01)
            await self.set_motor('stop', 0, False, False)
            self.set_speed(prev_speed)
        except Exception as e:
            print(e)
            await self.set_motor('stop', 0, False, False)

    async def exit(self):
        await self.set_motor('stop')
        self.pca.deinit()
