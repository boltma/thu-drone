#!/usr/bin/env python
#coding=UTF-8

import rospy
from pyservice_demo.srv import *

def client():
    rospy.init_node('greeting_pyclient', anonymous = False)
    rospy.wait_for_service('greetings')   #阻塞直至有server产生greetings
    try:
    	greetings_client = rospy.ServiceProxy('greetings', Greeting)
    	# resp = greetings_client('WANG', 22)
    	resp = greetings_client.call('WANG', 22)
    	rospy.loginfo('Message from server: %s' % resp.feedback)
    except rospy.ServiceException, e:
    	rospy.logwarn('Service call failed: %s' % e)

if __name__ == '__main__':
	client()
