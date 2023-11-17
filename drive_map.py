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
        self.direction = 0
        self.UP = 0
        self.DOWN = 1
        self.LEFT = 2
        self.RIGHT = 3

    def start(self):
        for i in range(len(self.map)):
            for j in range(len(self.map[i])):
                if self.map[i][j] == 8:
                    self.x = i
                    self.y = j
        print("starting thread")
        thr = threading.Thread(target=self.drive_map)
        thr.start()
    
    def drive_map(self):
        print(self.map)
        while not self.done:
            print(self.x, self.y)
            self.next(1)
            time.sleep(0.3)
        
    def next(self, distance):
        for i in range(distance):
            self.map[self.x][self.y] = 0
            try:
                if self.map[self.x-1][self.y] == 9:
                    self.x -= 1
                    if self.direction != self.UP:
                        if self.direction == self.LEFT:
                            self.drive.set_motor('turn_cw', 1)
                        elif self.direction == self.RIGHT:
                            self.drive.set_motor('turn_ccw', 1)
                        self.direction = self.UP
                    self.drive.set_motor('forward', 0, False)
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x+1][self.y] == 9:
                    self.x += 1
                    if self.direction != self.DOWN:
                        if self.direction == self.LEFT:
                            self.drive.set_motor('turn_ccw', 1)
                        elif self.direction == self.RIGHT:
                            self.drive.set_motor('turn_cw', 1)
                        self.direction = self.DOWN
                    self.drive.set_motor('forward', 0, False)
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x][self.y-1] == 9:
                    self.y -= 1
                    if self.direction != self.LEFT:
                        if self.direction == self.UP:
                            self.drive.set_motor('turn_ccw', 1)
                        elif self.direction == self.DOWN:
                            self.drive.set_motor('turn_cw', 1)
                        self.direction = self.LEFT
                    self.drive.set_motor('forward', 0, False)
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x][self.y+1] == 9:
                    self.y += 1
                    if self.direction != self.RIGHT:
                        if self.direction == self.UP:
                            self.drive.set_motor('turn_cw', 1)
                        elif self.direction == self.DOWN:
                            self.drive.set_motor('turn_ccw', 1)
                        self.direction = self.RIGHT
                    self.drive.set_motor('forward', 0, False)
                    continue
            except IndexError:
                pass
            self.drive.set_motor('stop')
            self.done = True

    def get_position(self):
        return self.x, self.y
