import requests
import json


class DriveTestClient:
    def __init__(self):
        self.url = 'http://192.168.0.3:1881'
        self.headers = {'Content-Type': 'application/json'}

    def manual_drive_request(self, direction, speed):
        assert direction in ['forward', 'backward', 'left', 'right', 'turn_cw', 'turn_ccw', 'stop']
        assert 0 <= speed <= 1
        req_data = {'action': direction, 'speed': speed}
        try:
            r = requests.post(
                self.url + '/drive',
                json=json.dumps(req_data),
                headers=self.headers)
            return json.loads(r.text)
        except Exception as e:
            print(e)
            return None


if __name__ == '__main__':
    client = DriveTestClient()
    speed = 0.1
    while True:
        key = input('Enter a command: ')
        if key == 'w':
            print(client.manual_drive_reqeust('forward', speed))
        elif key == 's':
            print(client.manual_drive_reqeust('backward', speed))
        elif key == 'a':
            print(client.manual_drive_reqeust('left', speed))
        elif key == 'd':
            print(client.manual_drive_reqeust('right', speed))
        elif key == 'q':
            print(client.manual_drive_reqeust('turn_ccw', speed))
        elif key == 'e':
            print(client.manual_drive_reqeust('turn_cw', speed))
        elif key == 't':
            speed = input("Enter speed: ")
        elif key == 'x':
            print(client.manual_drive_reqeust('stop', speed))
        else:
            print('invalid key')