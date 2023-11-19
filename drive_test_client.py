import requests
import json


class DriveTestClient:
    def __init__(self):
        self.url = 'http://192.168.0.3:1881'
        self.headers = {'Content-Type': 'application/json'}

    def manual_drive_request(self, direction, speed = None):
        assert direction in ['forward', 'backward', 'left', 'right', 'turn_cw', 'turn_ccw', 'stop']
        if speed is None:
            req_data = {'action': direction, 'duration': 2}
        else:
            req_data = {'action': direction, 'speed': speed, 'duration': 2}
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
            print(client.manual_drive_request('forward'))
        elif key == 's':
            print(client.manual_drive_request('backward'))
        elif key == 'a':
            print(client.manual_drive_request('left'))
        elif key == 'd':
            print(client.manual_drive_request('right'))
        elif key == 'q':
            print(client.manual_drive_request('turn_ccw'))
        elif key == 'e':
            print(client.manual_drive_request('turn_cw'))
        elif key == 't':
            speed = input("Enter speed: ")
            print(client.manual_drive_request('stop', speed)) 
        elif key == 'x':
            print(client.manual_drive_request('stop'))
        else:
            print('invalid key')
