#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import cv2
import tello_new as tello
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import threading
import time


def callback(data,drone):
	command=data.data
	drone.send_command(command)

def sub_ros():
	###########################
	#/command可以用来提前终止和调试
	###########################
	rospy.Subscriber("command",String,callback,drone)
	#rospy.spin()


def control():
	###########################
	#开发使用,加入你自己的代码，可以直接利用img和state信息
	###########################
        pass	
	#drone.send_command("takeoff")
	#drone.send_command("go 0 50 0 10")
	#drone.send_command("land")



global img,tello_state
img=None
tello_state=None
drone = tello.Tello('', 8888)
rospy.init_node('tello_state')
state_pub = rospy.Publisher('tello_state',String, queue_size=3)
img_pub = rospy.Publisher('tello_img',Image, queue_size=5)

sub_thread = threading.Thread(target=sub_ros)
sub_thread.start()

con_thread = threading.Thread(target=control)
con_thread.start()

try:
	while not rospy.is_shutdown():
		state=drone.read_state()
		if state is None or len(state) == 0:
			continue
		tello_state="".join(state)
		

		state_pub.publish(tello_state)
		frame=drone.read_frame()
		if frame is None or frame.size == 0:
			continue
		img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		cv2.imshow("img", img)
		cv2.waitKey(1)

		try:
			img_msg = CvBridge().cv2_to_imgmsg(img, 'bgr8')
			img_msg.header.frame_id = rospy.get_namespace()
		except CvBridgeError as err:
			rospy.logerr('fgrab: cv bridge failed - %s' % str(err))
			continue
		img_pub.publish(img_msg)
		
		
		#data=rospy.wait_for_message("command",String, timeout=None)
		#callback(data,drone)
except rospy.ROSInterruptException:
	pass

