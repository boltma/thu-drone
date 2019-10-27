#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import cv2
import tello_new as tello
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import threading
import random
import numpy as np
import sys

y_max_th = 200
y_min_th = 170

tello_state = None
tello_state_lock=threading.Lock()    
img = None
img_lock=threading.Lock()    

class control_handler:   #通过这个类发送控制指令到command话题
    def __init__(self,control_pub):
        self.control_pub = control_pub
    
    def forward(self, cm):
        command = "forward "+(str(cm))
        self.control_pub.publish(command)
    
    def back(self, cm):
        command = "back "+(str(cm))
        self.control_pub.publish(command)
    
    def up(self, cm):
        command = "up "+(str(cm))
        self.control_pub.publish(command)
    
    def down(self, cm):
        command = "down "+(str(cm))
        self.control_pub.publish(command)
    
    def right(self, cm):
        command = "right "+(str(cm))
        self.control_pub.publish(command)
    
    def left(self, cm):
        command = "left "+(str(cm))
        self.control_pub.publish(command)

    def cw(self, cm):
        command = "cw "+(str(cm))
        self.control_pub.publish(command)


    def ccw(self, cm):
        command = "ccw "+(str(cm))
        self.control_pub.publish(command)

    def takeoff(self):
        command = "takeoff"
        self.control_pub.publish(command)
        print ("ready")

    def land(self):
        command = "land"
        self.control_pub.publish(command)

    def stop(self):
        command = "stop"
        self.control_pub.publish(command)


class info_updater():   #初始化两个subscriber来订阅无人机的状态以及图像信息
    def __init__(self):
        rospy.Subscriber("tello_state",String,self.update_state)
        rospy.Subscriber("tello_img",Image, self.update_img)
        con_thread = threading.Thread(target=rospy.spin)
        con_thread.start()

    def update_state(self,data):
        global tello_state,tello_state_lock
        tello_state_lock.acquire()#线程锁
        tello_state = data.data
        tello_state_lock.release()
        # print(tello_state)

    def update_img(self,data):
        global img,img_lock
        img_lock.acquire()#线程锁
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding="passthrough")
        img_lock.release()
        # print(img)



def parse_state():#将接收到的字符串类型的状态信息放进字典中方便查询
    global tello_state,tello_state_lock
    tello_state_lock.acquire()
    statestr = tello_state.split(';')
    print (statestr)
    dict={}
    for item in statestr:
        if 'mid:' in item:
            mid = int(item.split(':')[-1])
            dict['mid'] = mid
        elif 'x:' in item:
            x = int(item.split(':')[-1])
            dict['x'] = x
        elif 'z:' in item:
            z = int(item.split(':')[-1])
            dict['z'] = z
        elif 'mpry:' in item:
            mpry = item.split(':')[-1]
            mpry = mpry.split(',')
            dict['mpry'] = [int(mpry[0]),int(mpry[1]),int(mpry[2])]
        # y 有可能被匹配成 mpry 所以放在后面
        elif 'y:' in item:
            y = int(item.split(':')[-1])
            dict['y'] = y
        elif 'pitch:' in item:
            pitch = int(item.split(':')[-1])
            dict['pitch'] = pitch
        elif 'roll:' in item:
            roll = int(item.split(':')[-1])
            dict['roll'] = roll
        elif 'yaw:' in item:
            yaw = int(item.split(':')[-1])
            dict['yaw'] = yaw
    tello_state_lock.release()
    return dict

def showimg():
    global img,img_lock
    img_lock.acquire()
    cv2.imshow("img",img)
    cv2.waitKey(30)
    img_lock.release()

class task_handle():#初步完成起飞识别定位毯，并飞行至中央
    class taskstages():
        finding_location  = 0 # 如果连定位毯都没找到就先找定位毯子
        order_location  = 1 # 找到定位毯之后把放到定位毯中心，朝向摆正
        finished = 6 # 结束

    def __init__(self , ctrl):
        self.States_Dict = None
        self.ctrl = ctrl
        self.now_stage = self.taskstages.finding_location

    def main(self): # 循环检测无人机是否完成任务，无人机移动到100
        while not (self.now_stage == self.taskstages.finished):
            if(self.now_stage == self.taskstages.finding_location):
                self.finding_location()
            elif(self.now_stage == self.taskstages.order_location):
                self.order_location()
        print("finished")
        exit(0)
    
    def finding_location(self): # 找到定位毯位置，一般来说，只要够高就可以识别到
        print ( "1234123" )
        assert (self.now_stage == self.taskstages.finding_location)
        while not ( parse_state()['mid'] > 0 ): # 如果没有找到定位毯，就一直尝试找
            distance = random.randint(20,30) # 上升距离随机产生
            print (distance)
            self.ctrl.up(distance) # 执行上升
            time.sleep(4) # 等待操作完成
            showimg()
        self.now_stage = self.taskstages.order_location

    def order_location(self):# 找到定位毯之后，通过状态的反馈信息进行控制，循环执行使得飞机达到毯子中央
        assert (self.now_stage == self.taskstages.order_location)
        state_conf = 0
        self.States_Dict = parse_state()
        while not ( self.States_Dict['yaw'] <= 8 and self.States_Dict['yaw'] >= -8 and self.States_Dict['x'] <= 120 and self.States_Dict['x'] >= 80 and  self.States_Dict['y'] <= 120 and self.States_Dict['y'] >= 80 and abs(self.States_Dict['z']) <= 150 and abs(self.States_Dict['z']) >= 190):
            if ( abs(self.States_Dict['z']) > 190 or abs(self.States_Dict['z']) < 150 ):
                if (self.States_Dict['z'] > 150):
                    self.ctrl.up(20)     
                    time.sleep(4)
                elif (self.States_Dict['z'] < 190):
                    self.ctrl.down(20) 
                    time.sleep(4)
            elif ( self.States_Dict['yaw'] < -8 or self.States_Dict['yaw'] > 8 ):
                if (self.States_Dict['yaw'] < -8):
                    self.ctrl.cw(10)
                    time.sleep(4)
                elif(self.States_Dict['yaw'] > 8):
                    self.ctrl.ccw(10)
                    time.sleep(4)
            elif ( self.States_Dict['x'] < 80 or self.States_Dict['x'] > 120 ):
                if (self.States_Dict['x'] < 80):
                    self.ctrl.right(20)
                    time.sleep(4)
                elif(self.States_Dict['x'] > 120):
                    self.ctrl.left(20)
                    time.sleep(4)
            elif ( self.States_Dict['y'] < 80 or self.States_Dict['y'] > 120 ):
                if (self.States_Dict['y'] < 80):
                    self.ctrl.back(20)
                    time.sleep(4)
                elif(self.States_Dict['y'] > 120):
                    self.ctrl.forward(20)
                    time.sleep(4)
            else:
                time.sleep(2)
                self.ctrl.stop()
                state_conf += 1
            self.States_Dict = parse_state()
            showimg()
            if self.States_Dict['mid'] < 0 :
                self.now_stage = self.taskstages.finding_location
                break
        self.now_stage = self.taskstages.finished     


if __name__ == '__main__':
    rospy.init_node('control_node', anonymous=True)
    control_pub = rospy.Publisher('command',String, queue_size=1)
    ctrl = control_handler(control_pub)
    img,tello_state
    infouper = info_updater()
    tasker = task_handle(ctrl)
    #################
    time.sleep(5.2)
    print ( 1)
    ctrl.takeoff( )
    print ( 4)
    time.sleep(4)
    ctrl.up(60)
    time.sleep(4)
    # # print ( 2)
    # # time.sleep(6)
    # # print ( 3)
    # # ctrl.left(20)
    # # print ( 4)
    # # time.sleep(6)
    # # ctrl.right(20)
    # # time.sleep(6)
    # # print ( 5)
    # # ctrl.forward(20)
    # # time.sleep(6)
    tasker.main()

    # time.sleep(1)
    # ctrl.forward(100)
    # time.sleep(6)
    # ctrl.back(100)
    # time.sleep(6)
    # ctrl.right(100)
    # time.sleep(6)
    # # ctrl.left(100)
    # time.sleep(6)
    # ctrl.land()

    

