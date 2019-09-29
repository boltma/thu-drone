#!/usr/bin/env python
#coding=utf-8

import rospy
import math
from pytopic_demo.msg import gps

def callback(gps):    # 回调函数输入的应该是msg
    distance = math.sqrt(math.pow(gps.x, 2) + math.pow(gps.y, 2))
    rospy.loginfo("Listener: GPS distance = %f, state: %s", distance, gps.state)

def listener():
    rospy.init_node('pylistener', anonymous=False)
    rospy.Subscriber('gps_info', gps, callback)    # Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型，第三个参数是回调函数的名称
    rospy.spin()

if __name__ == '__main__':
    listener()

