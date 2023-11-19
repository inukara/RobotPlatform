#!/usr/bin/env python3.7

import rospy
import requests
import json
import time
from sensor_msgs.msg import LaserScan

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #rospy.loginfo(len(data.ranges))
    #rospy.loginfo(data.ranges)
    sides = []
    for i in range(1, 4):
        idx = len(data.ranges)//4*i
        sides.append(min([d for d in data.ranges[idx-15:idx+15] if d > 0.0]))
        
    rospy.loginfo(sides)
    try:
        r = requests.post(
            'http://localhost:1881/lidar',
            json=json.dumps({'front': sides[1], 'right': sides[0], 'left': sides[2]}))
        if r.status_code != 200:
            rospy.loginfo(r.status_code)
    except Exception as e:
        pass
        rospy.loginfo('Connection error')
        time.sleep(1)
    

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
