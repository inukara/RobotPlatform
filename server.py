from quart import Quart, jsonify, request, abort, Response
import drive_map
import drive
import json

app = Quart(__name__)
dm = drive_map.DriveMap()
d = drive.Drive()

import logging
logging.getLogger('hypercorn.access').disabled = True

front_distance = 0
right_distance = 0
left_distance = 0

# action: forward, backward, left, right, turn_cw, turn_ccw, stop
# speed: 0 <= speed <= 1
# duration: motor run time in seconds
@app.route('/drive', methods=['POST'])
async def robot_drive():
    req = json.loads(await request.get_json())
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
        await d.set_motor(action, duration)
    elif action == 'exit':
        await d.exit()
    else:
        abort(400)
    return jsonify({'status': 'success'})


@app.route('/lidar', methods=['POST'])
async def lidar():
    req = json.loads(await request.get_json())
    front_distance = float(req['front'])
    right_distance = float(req['right'])
    left_distance = float(req['left'])
    front_point = float(req['front_point'])
    if front_distance < 0.7 and front_distance != 0 and dm.obstacle_blocks == 0:
        await d.set_motor("stop")
        #print(front_distance, "front distance unsafe")
        dm.done = True
        dm.obstacle = True
        dm.obstacle_blocks = 5
    elif False: #right_distance < 1.0 and right_distance != 0:
        await d.set_motor("stop")
        print(right_distance, "right distance unsafe")
        dm.done = True
        dm.obstacle = True
    elif False: #left_distance < 1.0 and left_distance != 0:
        await d.set_motor("stop")
        print(left_distance, "left distance unsafe")
        dm.done = True
        dm.obstacle = True
    #d.calibrate(right_distance)
    dm.cur_dist = front_point
    return Response(status=200)


@app.route('/imu', methods=['POST'])
async def imu():
    req = json.loads(await request.get_json())
    x = float(req['x'])
    y = float(req['y'])
    z = float(req['z'])
    w = float(req['w'])
    drive.imu = [x, y, z, w]
    #print(d.imu)
    return Response(status=200)


@app.route('/degree', methods=['POST'])
async def degree():
    req = json.loads(await request.get_json())
    degrees = float(req['degrees'])
    direction = req['direction']
    await d.turn_degrees(degrees, direction)
    return jsonify({'status': 'success'})


# below is the code for the car drive simulation
# start the car drive simulation
@app.route('/start', methods=['POST'])
async def start():
    dm.done = False
    dm.obstacle = False
    await dm.start()
    return jsonify({'status': 'success'})


@app.route('/reset', methods=['POST'])
async def reset():
    dm.reset()
    return jsonify({'status': 'success'})


# get the current position of the car
@app.route('/position')
async def get_position():
    x, y = dm.get_position()
    return jsonify({'x': x, 'y': y})


@app.route('/obstacle', methods=['GET'])
async def get_obstacle():
    if dm.obstacle:
        return jsonify({'obstacle': dm.obstacle, 'map': dm.map})
    else:
        return jsonify({'obstacle': dm.obstacle, 'map': []})


# get map data array in json format
@app.route('/map', methods=['POST'])
async def get_map():
    req = json.loads(await request.get_json())
    m = json.loads(req['map'])
    for i in range(len(m)):
        for j in range(len(m[i])):
            print(m[i][j], end=' ')
        print()
    if m is None:
        abort(400)
    dm.map = m
    return jsonify({'status': 'success'})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=1881)
