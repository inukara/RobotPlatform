#!/usr/bin/env python3.7

import rospy
import requests
import json
from sensor_msgs.msg import LaserScan

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #rospy.loginfo(len(data.ranges))
    #rospy.loginfo(data.ranges)
    front = data.ranges[len(data.ranges)//2]
    rospy.loginfo(front)
    r = requests.post('http://localhost:1881/lidar', json=json.dumps({'distance': front}))
    rospy.loginfo(r.status_code)
    

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()