
class DriveSim:
    def __init__(self):
        self.map = []
        self.x = 0
        self.y = 0

    def start(self):
        for i in range(len(self.map)):
            for j in range(len(self.map[i])):
                if self.map[i][j] == 8:
                    self.x = i
                    self.y = j
                    return

    def next(self, distance):
        for i in range(distance):
            self.map[self.x][self.y] = 0
            try:
                if self.map[self.x+1][self.y] == 9:
                    self.x += 1
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x-1][self.y] == 9:
                    self.x -= 1
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x][self.y+1] == 9:
                    self.y += 1
                    continue
            except IndexError:
                pass
            try:
                if self.map[self.x][self.y-1] == 9:
                    self.y -= 1
                    continue
            except IndexError:
                pass

    def get_position(self):
        return self.x, self.y
