from flask import Flask, jsonify, request, abort, Response
import drive_map
import drive
import json

app = Flask(__name__)
dm = drive_map.DriveMap()
d = drive.Drive()

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

front_distance = 0
right_distance = 0
left_distance = 0


# action: forward, backward, left, right, turn_cw, turn_ccw, stop
# speed: 0 <= speed <= 1
# duration: motor run time in seconds
@app.route('/drive', methods=['POST'])
def robot_drive():
    req = json.loads(request.get_json())
    action = req['action']
    try:
        duration = float(req['duration'])
        assert action in d.speeds
        if req['speed'] is not None:
            speed = float(req['speed'])
            assert 0 <= speed <= 1
            d.set_speed(speed)
    except ValueError:
        abort(400)
    except AssertionError:
        abort(400)
    except KeyError:
        pass
    if action in d.speeds:
        d.set_motor(action, duration)
    elif action == 'exit':
        d.exit()
    else:
        abort(400)
    return jsonify({'status': 'success'})


@app.route('/lidar', methods=['POST'])
def lidar():
    req = json.loads(request.get_json())
    front_distance = float(req['front'])
    right_distance = float(req['right'])
    left_distance = float(req['left'])
    if front_distance < 1.0 and front_distance != 0:
        d.set_motor("stop")
        #print(front_distance, "front distance unsafe")
        dm.done = True
    elif False: #right_distance < 1.0 and right_distance != 0:
        d.set_motor("stop")
        print(right_distance, "right distance unsafe")
        dm.done = True
    elif False: #left_distance < 1.0 and left_distance != 0:
        d.set_motor("stop")
        print(left_distance, "left distance unsafe")
        dm.done = True
    d.calibrate(right_distance)
    dm.cur_dist = front_distance
    return Response(status=200)


# below is the code for the car drive simulation
# start the car drive simulation
@app.route('/start', methods=['POST'])
def start():
    dm.done = False
    dm.start()
    return jsonify({'status': 'success'})


# get the current position of the car
@app.route('/position')
def get_position():
    x, y = dm.get_position()
    return jsonify({'x': x, 'y': y})


# get map data array in json format
@app.route('/map', methods=['POST'])
def get_map():
    req = json.loads(request.get_json())
    m = json.loads(req['map'])
    print(m)
    print(type(m))
    if m is None:
        abort(400)
    dm.map = m
    return jsonify({'status': 'success'})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=1881)
