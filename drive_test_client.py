import requests
import json


class DriveTestClient:
    def __init__(self):
        self.url = 'http://192.168.0.3:1881'
        self.headers = {'Content-Type': 'application/json'}

    def request(self, action):
        req_data = {'action': action}
        try:
            r = requests.post(self.url + '/drive', json=json.dumps(req_data), headers=self.headers)
            return json.loads(r.text)
        except Exception as e:
            print(e)
            return None


if __name__ == '__main__':
    client = DriveTestClient()
    while True:
        key = input('Enter a command: ')
        if key == 'w':
            print(client.request('forward'))
        elif key == 's':
            print(client.request('backward'))
        elif key == 'a':
            print(client.request('left'))
        elif key == 'd':
            print(client.request('right'))
        elif key == 'q':
            print(client.request('turn_ccw'))
        elif key == 'e':
            print(client.request('turn_cw'))
        elif key == 'x':
            print(client.request('stop'))
        else:
            print('invalid key')