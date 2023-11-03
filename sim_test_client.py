import requests
import json


class DriveClient:
    def __init__(self):
        self.url = ''
        self.headers = {'Content-Type': 'application/json'}
        # 바로 밑 줄 지우고 맞는 조 이름 입력
        self.id = 'test'

    # 현재 위치 반환
    # return: {'x': x, 'y': y}
    def get_position(self):
        try:
            r = requests.get(self.url + f'/position?id={self.id}', headers=self.headers)
            return json.loads(r.text)
        except Exception as e:
            print(e)
            return None

    # 지도 데이터 전송
    # class init에 조 이름 확인할것
    # map_data는 2차원 배열
    def map(self, map_data):
        map_request_data = {'id': f'{self.id}', 'map': json.dumps(map_data)}
        #print(map_request_data)
        #print(map_request_data['map'])

        try:
            r = requests.post(self.url + '/map', json=json.dumps(map_request_data), headers=self.headers)
            #return json.loads(r.text)
            return r.text
        except Exception as e:
            print(e)
            return None

    # 주행 시작 좌표 입력
    # x가 세로 y가 가로
    def start(self, x, y):
        start_request_data = {'id': f'{self.id}', 'x': x, 'y': y}
        try:
            r = requests.post(self.url + '/start', json=json.dumps(start_request_data), headers=self.headers)
            return json.loads(r.text)
        except Exception as e:
            print(e)
            return None

    # 지정된 칸 수만큼 진행
    def next(self, distance):
        req_data = {'id': f'{self.id}', 'distance': distance}
        try:
            r = requests.post(self.url + '/next', json=json.dumps(req_data), headers=self.headers)
            return json.loads(r.text)
        except Exception as e:
            print(e)
            return None

if __name__ == '__main__':
    while True:
        drive = DriveClient()
        s = input('Enter a command: ')
        if s == 'position':
            print(drive.get_position())
        elif s == 'start':
            x = int(input('Enter x position: '))
            y = int(input('Enter y position: '))
            print(drive.start(x, y))
        elif s == 'next':
            distance = int(input('Enter a distance: '))
            print(drive.next(distance))
        elif s == 'map':
            # 여기에 맵 데이터 입력
            m = [[9, 0, 0, 0, 0],
                 [9, 1, 1, 1, 0],
                 [9, 1, 9, 1, 0],
                 [9, 1, 9, 1, 0],
                 [9, 9, 9, 0, 0]]
            print(drive.map(m))

        else:
            print('Invalid command')
