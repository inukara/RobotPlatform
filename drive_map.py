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
        i = 0
        while not self.done:
            print(i)
            i = i + 1
            print(self.x, self.y)
            self.next(1)
            time.sleep(0.3)
        
    def next(self, distance):
        for i in range(distance):
            self.map[self.x][self.y] = 0
            try:
                if self.map[self.x-1][self.y] == 9:
                    self.x -= 1
                    self.drive.set_motor('forward', 0.1, 0, False)
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x+1][self.y] == 9:
                    self.x += 1
                    self.drive.set_motor('backward', 0.1, 0, False)
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x][self.y-1] == 9:
                    self.y -= 1
                    self.drive.set_motor('left', 0.1, 0, False)
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x][self.y+1] == 9:
                    self.y += 1
                    self.drive.set_motor('right', 0.1, 0, False)
                    continue
            except IndexError:
                pass
            self.drive.set_motor('stop')
            self.done = True

    def get_position(self):
        return self.x, self.y
