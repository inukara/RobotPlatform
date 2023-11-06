from flask import Flask, jsonify, request, abort
import drivesim
import drive
import json

app = Flask(__name__)
sim = drivesim.DriveSim()
d = drive.Drive()


@app.route('/drive', methods=['POST'])
def drive():
    req = json.loads(request.get_json())
    action = req['action']
    try:
        speed = float(req['speed'])
        assert 0 <= speed <= 1
        assert action in d.speeds
    except ValueError:
        abort(400)
    except AssertionError:
        abort(400)
    if action in d.speeds:
        d.set_motor(action, speed)
    elif action == 'exit':
        d.exit()
    else:
        abort(400)
    return jsonify({'status': 'success'})


# below is the code for the car drive simulation
# start the car drive simulation
@app.route('/start', methods=['POST'])
def start():
    req = json.loads(request.get_json())
    if req['id'] is None:
        abort(400)
    sim.start()
    x = sim.x
    y = sim.y
    return jsonify({'status': 'success', 'x': x, 'y': y})


# go to the next adjacent cell
@app.route('/next', methods=['POST'])
def next():
    req = json.loads(request.get_json())
    if req['id'] is None:
        abort(400)
    distance = req['distance']
    x, y = -1, -1
    if distance is None:
        abort(400)
    print(sim.map)
    sim.next(distance)
    x = sim.x
    y = sim.y
    return jsonify({'status': 'success', 'x': x, 'y': y})


# get the current position of the car
@app.route('/position')
def get_position():
    arg = str(request.args.get('id'))
    if arg is None:
        abort(400)
    x, y = sim.get_position()
    return jsonify({'x': x, 'y': y})


# get map data array in json format
@app.route('/map', methods=['POST'])
def get_map():
    req = json.loads(request.get_json())
    if req['id'] is None:
        abort(400)
    m = json.loads(req['map'])
    print(m)
    print(type(m))
    if m is None:
        abort(400)
    sim.map = m
    return jsonify({'status': 'success'})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=1881)
