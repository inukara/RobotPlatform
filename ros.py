#!/usr/bin/env python3.7

import rospy
import requests
import json
import time
from sensor_msgs.msg import LaserScan, Imu

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #rospy.loginfo(len(data.ranges))
    #rospy.loginfo(data.ranges)
    sides = []
    for i in range(1, 4):
        idx = len(data.ranges)//4*i
        sides.append(min([d for d in data.ranges[idx-20:idx+20] if d > 0.0]))
        
    rospy.loginfo(sides)
    try:
        r = requests.post(
            'http://localhost:1881/lidar',
            json=json.dumps({'front': sides[1], 'right': sides[0], 'left': sides[2], 'front_point': data.ranges[len(data.ranges)//2]}))
        if r.status_code != 200:
            rospy.loginfo(r.status_code)
    except Exception as e:
        pass
        rospy.loginfo('Connection error, lidar')


def imucallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    data = {
        'x': data.orientation.x,
        'y': data.orientation.y,
        'z': data.orientation.z,
        'w': data.orientation.w
    }
    try:
        r = requests.post(
            'http://localhost:1881/imu',
            json=json.dumps(data))
        if r.status_code != 200:
            rospy.loginfo(r.status_code)
    except Exception as e:
        pass
        rospy.loginfo('Connection error, imu')
    

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.Subscriber("/imu/data_throttle", Imu, imucallback)
    rospy.spin()

if __name__ == '__main__':
    listener()
