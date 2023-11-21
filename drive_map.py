import drive
import threading
import time

class DriveMap:
    def __init__(self):
        self.map = []
        self.x = 0
        self.y = 0
        self.drive = drive.Drive()
        self.done = False
        self.direction = -1
        self.UP = 0
        self.DOWN = 1
        self.LEFT = 2
        self.RIGHT = 3
        self.prev_dist = 0.0
        self.cur_dist = 0.0
        self.TURN_WAIT = 2
        # medical = 2.5
        self.MOTOR_TURN_DELAY = 2.5 # temporary
        self.FIRST_TURN = True

    def start(self):
        # variable reset
        for i in range(len(self.map)):
            for j in range(len(self.map[i])):
                if self.map[i][j] == 8:
                    self.x = i
                    self.y = j
        self.done = False
        self.direction = -1
        self.FIRST_TURN = True
        
        print(f"{self.x} {self.y} starting thread")
        thr = threading.Thread(target=self.drive_map)
        thr.start()
    
    def drive_map(self):
        print(self.map)
        if self.prev_dist == 0.0:
            self.prev_dist = self.cur_dist
        self.drive.set_motor('forward', 0, False)
        while not self.done:
            #print(f"distance: {self.prev_dist} {self.cur_dist}")
            '''
            if self.prev_dist < self.cur_dist:
                print("prev_distance error, fixing")
                self.prev_dist = self.cur_dist
            '''
            if self.prev_dist - self.cur_dist > 0.2:
                print(self.x, self.y)
                self.next(1)
                self.prev_dist = self.cur_dist

    def after_turn(self):
        time.sleep(self.MOTOR_TURN_DELAY + self.TURN_WAIT)
        self.drive.set_motor('forward', 0, False)
        self.prev_dist = self.cur_dist

    def next(self, distance):
        for i in range(distance):
            self.map[self.x][self.y] = 0
            try:
                if self.map[self.x-1][self.y] == 9:
                    self.x -= 1
                    if self.direction != self.UP:
                        if self.FIRST_TURN:
                            self.FIRST_TURN = False
                            self.direction = self.UP
                            continue
                        print("turning north")
                        self.drive.set_motor('stop')
                        time.sleep(self.TURN_WAIT)
                        if self.direction == self.LEFT:
                            self.drive.set_motor('turn_cw', self.MOTOR_TURN_DELAY)
                        elif self.direction == self.RIGHT:
                            self.drive.set_motor('turn_ccw', self.MOTOR_TURN_DELAY)
                        print("done turning")
                        self.direction = self.UP
                        self.after_turn()
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x+1][self.y] == 9:
                    self.x += 1
                    if self.direction != self.DOWN:
                        if self.FIRST_TURN:
                            self.FIRST_TURN = False
                            self.direction = self.DOWN
                            continue
                        print("turning south")
                        self.drive.set_motor('stop')
                        time.sleep(self.TURN_WAIT)
                        if self.direction == self.LEFT:
                            self.drive.set_motor('turn_ccw', self.MOTOR_TURN_DELAY)
                        elif self.direction == self.RIGHT:
                            self.drive.set_motor('turn_cw', self.MOTOR_TURN_DELAY)
                        print("done turning")
                        self.direction = self.DOWN
                        self.after_turn()
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x][self.y-1] == 9:
                    self.y -= 1
                    if self.direction != self.LEFT:
                        if self.FIRST_TURN:
                            self.FIRST_TURN = False
                            self.direction = self.LEFT
                            continue
                        print("turning west")
                        self.drive.set_motor('stop')
                        time.sleep(self.TURN_WAIT)
                        if self.direction == self.UP:
                            self.drive.set_motor('turn_ccw', self.MOTOR_TURN_DELAY)
                        elif self.direction == self.DOWN:
                            self.drive.set_motor('turn_cw', self.MOTOR_TURN_DELAY)
                        print("done turning")
                        self.direction = self.LEFT
                        self.after_turn()
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x][self.y+1] == 9:
                    self.y += 1
                    if self.direction != self.RIGHT:
                        if self.FIRST_TURN:
                            self.FIRST_TURN = False
                            self.direction = self.RIGHT
                            continue
                        print("turning east")
                        self.drive.set_motor('stop')
                        time.sleep(self.TURN_WAIT)
                        if self.direction == self.UP:
                            self.drive.set_motor('turn_cw', self.MOTOR_TURN_DELAY)
                        elif self.direction == self.DOWN:
                            self.drive.set_motor('turn_ccw', self.MOTOR_TURN_DELAY)
                        print("done turning")
                        self.direction = self.RIGHT
                        self.after_turn()
                    continue
            except IndexError:
                pass
            print("stopping")
            self.drive.set_motor('stop')
            self.done = True

    def get_position(self):
        return self.x, self.y
