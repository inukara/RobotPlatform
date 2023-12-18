import drive
import asyncio

class DriveMap:
    def __init__(self):
        self.map = []
        self.x = 0
        self.y = 0
        self.drive = drive.Drive()
        self.done = False
        self.INIT_DIRECTION = 0
        # medical = 1
        # h3m = 0
        self.direction = self.INIT_DIRECTION
        self.UP = 0
        self.DOWN = 1
        self.LEFT = 2
        self.RIGHT = 3
        self.prev_dist = 0.0
        self.cur_dist = 0.0
        self.TURN_WAIT = 1
        self.obstacle = False
        self.obstacle_blocks = 0

    async def start(self):
        # variable reset
        for i in range(len(self.map)):
            for j in range(len(self.map[i])):
                if self.map[i][j] == 8:
                    self.x = i
                    self.y = j
        self.done = False
        self.prev_dist = 0.0
        
        print(f"{self.x} {self.y} starting thread")
        await self.next()
        asyncio.create_task(self.drive_map())
    
    def reset(self):
        self.map = []
        self.x = 0
        self.y = 0
        self.done = False
        self.direction = self.INIT_DIRECTION
        self.prev_dist = 0.0
        self.cur_dist = 0.0
        self.obstacle = False
        self.obstacle_blocks = 0
    
    async def drive_map(self):
        if self.prev_dist == 0.0 and self.cur_dist != 0.0:
            self.prev_dist = self.cur_dist
        # self.drive.set_motor('forward', 0, False)
        while not self.done:
            #print(f"distance: {self.prev_dist} {self.cur_dist}")

            if self.prev_dist + 0.10 < self.cur_dist:
                print(f"distance error, prev:{self.prev_dist} < cur:{self.cur_dist} - not fixing")
                # self.prev_dist = self.cur_dist
            
            if self.prev_dist - self.cur_dist > 0.15:
                print(self.x, self.y)
                await self.next()
                self.prev_dist = self.cur_dist
                if self.obstacle_blocks > 0:
                    self.obstacle_blocks -= 1
            if self.obstacle:
                print("obstacle detected, stopping")
                await self.drive.set_motor('stop')
                self.obstacle = False
                self.obstacle_blocks = 5
                return
            await asyncio.sleep(0.01)

    async def next(self):
        self.map[self.x][self.y] = 0
        try:
            if self.map[self.x-1][self.y] == 9:
                self.x -= 1
                if self.direction != self.UP:
                    print("turning north")
                    await self.drive.set_motor('stop')
                    await asyncio.sleep(self.TURN_WAIT)
                    if self.direction == self.LEFT:
                        await self.drive.turn_degrees(90, 'cw')
                    elif self.direction == self.RIGHT:
                        await self.drive.turn_degrees(90, 'ccw')
                    elif self.direction == self.DOWN:
                        await self.drive.turn_degrees(90, 'cw')
                        await self.drive.turn_degrees(90, 'cw', True)
                    print("done turning")
                    self.direction = self.UP
                    await self.drive.set_motor('forward', 0, False)
                    self.prev_dist = self.cur_dist
                return
        except IndexError:
            pass
        try:
            if self.map[self.x+1][self.y] == 9:
                self.x += 1
                if self.direction != self.DOWN:
                    print("turning south")
                    await self.drive.set_motor('stop')
                    await asyncio.sleep(self.TURN_WAIT)
                    if self.direction == self.LEFT:
                        await self.drive.turn_degrees(90, 'ccw')
                    elif self.direction == self.RIGHT:
                        await self.drive.turn_degrees(90, 'cw')
                    elif self.direction == self.UP:
                        await self.drive.turn_degrees(90, 'cw')
                        await self.drive.turn_degrees(90, 'cw', True)
                    print("done turning")
                    self.direction = self.DOWN
                    await self.drive.set_motor('forward', 0, False)
                    self.prev_dist = self.cur_dist
                return
        except IndexError:
            pass
        try:
            if self.map[self.x][self.y-1] == 9:
                self.y -= 1
                if self.direction != self.LEFT:
                    print("turning west")
                    await self.drive.set_motor('stop')
                    await asyncio.sleep(self.TURN_WAIT)
                    if self.direction == self.UP:
                        await self.drive.turn_degrees(90, 'ccw')
                    elif self.direction == self.DOWN:
                        await self.drive.turn_degrees(90, 'cw')
                    elif self.direction == self.RIGHT:
                        await self.drive.turn_degrees(90, 'cw')
                        await self.drive.turn_degrees(90, 'cw', True)
                    print("done turning")
                    self.direction = self.LEFT
                    await self.drive.set_motor('forward', 0, False)
                    self.prev_dist = self.cur_dist
                return
        except IndexError:
            pass
        try:
            if self.map[self.x][self.y+1] == 9:
                self.y += 1
                if self.direction != self.RIGHT:
                    print("turning east")
                    await self.drive.set_motor('stop')
                    await asyncio.sleep(self.TURN_WAIT)
                    if self.direction == self.UP:
                        await self.drive.turn_degrees(90, 'cw')
                    elif self.direction == self.DOWN:
                        await self.drive.turn_degrees(90, 'ccw')
                    elif self.direction == self.LEFT:
                        await self.drive.turn_degrees(90, 'cw')
                        await self.drive.turn_degrees(90, 'cw', True)
                    print("done turning")
                    self.direction = self.RIGHT
                    await self.drive.set_motor('forward', 0, False)
                    self.prev_dist = self.cur_dist
                return
        except IndexError:
            pass
        print("stopping")
        await self.drive.set_motor('stop')
        self.done = True

    def get_position(self):
        return self.x, self.y
