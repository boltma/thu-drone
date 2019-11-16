#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import threading
import random
import os
import numpy as np

import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from detect import *

# if you can not find cv2 in your python, you can try this. usually happen when you use conda.
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

# import tello_base as tello

# basketball football volleyball balloon
_id = 0

y_max_th = 200
y_min_th = 170

img = None
tello_state = 'mid:-1;x:100;y:100;z:-170;mpry:1,180,1;pitch:0;roll:0;yaw:-19;'
tello_state_lock = threading.Lock()
img_lock = threading.Lock()
data = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "mtx.npz"))
mtx = data['mtx']
dist = data['dist']
model, device = load_weight()


def calib(imgSrc):
    if imgSrc is None:
        return
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0.1, (w, h))
    # 纠正畸变
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    return dst


def FindCircle(imgSrc):
    imgSrc = calib(imgSrc)
    imgSrcClone = np.copy(imgSrc)
    row, col = imgSrc.shape[:2]

    img = cv2.GaussianBlur(imgSrc, (5, 5), 0)
    lower_red = np.array([0, 0, 100])
    higher_red = np.array([60, 60, 255])
    red_range = cv2.inRange(img, lower_red, higher_red)
    imgRed = cv2.bitwise_and(img, img, mask=red_range)
    imgGray = cv2.cvtColor(imgRed, cv2.COLOR_BGR2GRAY)
    _, imgBinary = cv2.threshold(imgGray, 30, 255, cv2.THRESH_BINARY)
    # cv2.imshow("s", imgBinary)
    # cv2.waitKey(0)

    # erosion = cv2.morphologyEx(img, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

    circles = cv2.HoughCircles(imgBinary, cv2.HOUGH_GRADIENT, 2, 100, param1=100, param2=30, minRadius=10,
                               maxRadius=100)

    if circles is None:
        print("Circle not found")
        return 0

    circles = np.round(circles[0, :]).astype("int")

    for (x, y, r) in circles:
        # print(x, y, r)
        print('###')
        print(x, y, r)
        print('###')
        color = imgSrcClone[y][x]
        if color[0] < 20 and color[1] < 20 and color[2] > 80:
            if y < row / 3:
                if x < col / 2:
                    return 1
                else:
                    return 2
            else:
                if x < col / 2:
                    return 3
                else:
                    return 4
            # cv2.circle(imgSrcClone, (x, y), r, (0, 255, 0), 4)

    # cv2.imshow("contour", imgSrcClone)
    # cv2.waitKey(0)

    return 0


def FindBall(imgSrc, _id):
    imgSrc = calib(imgSrc)
    row, col = imgSrc.shape[:2]
    result = detect_ball(model, device, imgSrc)
    for ball in result:
        if int(ball[6].item()) == _id:
            x = (ball[0].item() + ball[2].item()) / 2
            # y = (ball[1].item() + ball[3].item()) / 2
            if x < col / 3:
                return 1
            elif col / 3 <= x <= 2 * col / 3:
                return 2
            else:
                return 3
            pass
    return 0


# send command to tello
class control_handler:
    def __init__(self, control_pub):
        self.control_pub = control_pub

    def forward(self, cm):
        command = "forward " + (str(cm))
        self.control_pub.publish(command)

    def back(self, cm):
        command = "back " + (str(cm))
        self.control_pub.publish(command)

    def up(self, cm):
        command = "up " + (str(cm))
        self.control_pub.publish(command)

    def down(self, cm):
        command = "down " + (str(cm))
        self.control_pub.publish(command)

    def right(self, cm):
        command = "right " + (str(cm))
        self.control_pub.publish(command)

    def left(self, cm):
        command = "left " + (str(cm))
        self.control_pub.publish(command)

    def cw(self, cm):
        command = "cw " + (str(cm))
        self.control_pub.publish(command)

    def ccw(self, cm):
        command = "ccw " + (str(cm))
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


# subscribe tello_state and tello_image
class info_updater():
    def __init__(self):
        rospy.Subscriber("tello_state", String, self.update_state)
        rospy.Subscriber("tello_image", Image, self.update_img)
        self.con_thread = threading.Thread(target=rospy.spin)
        self.con_thread.start()

    def update_state(self, data):
        global tello_state, tello_state_lock
        tello_state_lock.acquire()  # thread locker
        tello_state = data.data
        tello_state_lock.release()
        # print(tello_state)

    def update_img(self, data):
        global img, img_lock
        img_lock.acquire()  # thread locker
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding="passthrough")
        img_lock.release()
        # print(img)


# put string into dict, easy to find
def parse_state():
    global tello_state, tello_state_lock
    tello_state_lock.acquire()
    statestr = tello_state.split(';')
    print (statestr)
    dict = {}
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
            dict['mpry'] = [int(mpry[0]), int(mpry[1]), int(mpry[2])]
        # y can be recognized as mpry, so put y first
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
    global img, img_lock
    img_lock.acquire()
    cv2.imshow("tello_image", img)
    cv2.waitKey(30)
    img_lock.release()


# mini task: take off and fly to the center of the blanket.
class task_handle():
    class taskstages():
        finding_location = 0  # find locating blanket
        order_location = 1  # find the center of locating blanket and adjust tello
        finding_fire = 2
        passing_fire = 3
        finding_ball = 4
        passing_ball = 5
        finished = 6  # task done signal

    def __init__(self, ctrl):
        self.States_Dict = None
        self.ctrl = ctrl
        self.passed_fire = False
        self.fire_code = -1
        self.now_stage = self.taskstages.finding_location

    def main(self):  # main function: examine whether tello finish the task
        while not (self.now_stage == self.taskstages.finished):
            if self.now_stage == self.taskstages.finding_location:
                self.finding_location()
            elif self.now_stage == self.taskstages.order_location:
                self.order_location()
            elif self.now_stage == self.taskstages.finding_fire:
                code = FindCircle(img)
                print(code)
                if code != 0:
                    self.fire_code = code
                    self.order_location()
                else:
                    self.ctrl.up(20)
                    time.sleep(4)
                    self.now_stage = self.taskstages.order_location
            elif self.now_stage == self.taskstages.passing_fire:
                self.pass_fire()
            elif self.now_stage == self.taskstages.finding_ball:
                code = FindBall(img, _id)
                print(code)
                if code == 1:
                    self.ctrl.left(30)
                    time.sleep(4)
                if code == 3:
                    self.ctrl.right(30)
                    time.sleep(4)
                if code == 1 or code == 2 or code == 3:
                    self.ctrl.down(100)
                    time.sleep(10)
                    self.ctrl.forward(150)
                    time.sleep(10)

                    # self.order_location({'x': 300, 'y': 50, 'z': 110, 'mpry': 90})
                    # self.ctrl.cw(359)
                    # time.sleep(4)
                # if code == 2:
                #     self.order_location({'x': 300, 'y': 100, 'z': 110, 'mpry': 90})
                #     pass
                #     # self.ctrl.forward(30)
                #     # time.sleep(4)
                #     # self.ctrl.cw(359)
                #     # time.sleep(4)
                # if code == 3:
                #     self.order_location({'x': 300, 'y': 150, 'z': 110, 'mpry': 90})
                #     # self.ctrl.cw(359)
                #     # time.sleep(4)
                else:
                    self.now_stage = self.taskstages.order_location
                    continue
                # self.ctrl.up(100)
                # time.sleep(8)
                # self.ctrl.forward(200)
                # time.sleep(15)
                self.now_stage = self.taskstages.finished
        self.ctrl.land()
        print("Task Done!")

    def finding_location(self):  # find locating blanket (the higher, the easier)
        assert (self.now_stage == self.taskstages.finding_location)
        while not (parse_state()['mid'] > 0):  # if no locating blanket is found:
            distance = random.randint(20, 30)  # randomly select distance
            print (distance)
            self.ctrl.up(distance)  # tello up
            time.sleep(4)  # wait for command finished
            showimg()
        self.now_stage = self.taskstages.order_location

    def order_location(self, location=None):  # adjust tello to the center of locating blanket
        if location is None:
            if self.passed_fire:
                location = {'x': 300, 'y': 100, 'z': 170, 'mpry': 90}
            elif self.fire_code != -1:
                code = self.fire_code
                if code == 1:
                    location = {'x': 135, 'y': 75, 'z': 204, 'mpry': 90}
                elif code == 2:
                    location = {'x': 135, 'y': 130, 'z': 204, 'mpry': 90}
                elif code == 3:
                    location = {'x': 135, 'y': 75, 'z': 153, 'mpry': 90}
                elif code == 4:
                    location = {'x': 135, 'y': 130, 'z': 153, 'mpry': 90}
            else:
                location = {'x': 100, 'y': 100, 'z': 170, 'mpry': 90}
        # assert (self.now_stage == self.taskstages.order_location)
        state_conf = 0
        x = location['x']
        y = location['y']
        z = location['z']
        mpry = location['mpry']
        # yaw = location['yaw']
        self.States_Dict = parse_state()
        while not (mpry + 8 >= self.States_Dict['mpry'][1] + 90 >= mpry - 8 and
                   x + 20 >= self.States_Dict['x'] >= x - 20 and
                   y + 5 >= self.States_Dict['y'] >= y - 5 and
                   z + 10 >= abs(self.States_Dict['z']) >= z - 10):
            if abs(self.States_Dict['z']) > z + 20 or abs(self.States_Dict['z']) < z - 20:
                if abs(self.States_Dict['z']) < z - 20:
                    self.ctrl.up(20)
                    time.sleep(4)
                elif abs(self.States_Dict['z']) > z + 20:
                    self.ctrl.down(20)
                    time.sleep(4)
            elif abs(self.States_Dict['z']) > z + 10 or abs(self.States_Dict['z']) < z - 10:
                if abs(self.States_Dict['z']) > z + 10:
                    self.ctrl.up(20)
                    time.sleep(4)
                    self.ctrl.down(30)
                    time.sleep(4)
                elif abs(self.States_Dict['z']) < z - 10:
                    self.ctrl.up(30)
                    time.sleep(4)
                    self.ctrl.down(20)
                    time.sleep(4)
            elif self.States_Dict['mpry'][1] + 90 < mpry - 8 or self.States_Dict['mpry'][1] + 90 > mpry + 8:
                if self.States_Dict['mpry'][1] + 90 < mpry - 8:
                    self.ctrl.cw(10)
                    time.sleep(4)
                elif self.States_Dict['mpry'][1] + 90 > mpry + 8:
                    self.ctrl.ccw(10)
                    time.sleep(4)
            elif self.States_Dict['x'] < x - 20 or self.States_Dict['x'] > x + 20:
                if self.States_Dict['x'] < x - 20:
                    self.ctrl.forward(20)
                    time.sleep(4)
                elif self.States_Dict['x'] > x + 20:
                    self.ctrl.back(20)
                    time.sleep(4)
            # elif x - 20 <= self.States_Dict['x'] < x - 5 or x + 20 >= self.States_Dict['x'] > x + 5:
            #     if self.States_Dict['x'] < x - 5:
            #         self.ctrl.forward(5)
            #         time.sleep(4)
            #     elif self.States_Dict['x'] > x + 5:
            #         self.ctrl.back(5)
            #         time.sleep(4)
            elif self.States_Dict['y'] < y - 20 or self.States_Dict['y'] > y + 20:
                if self.States_Dict['y'] < y - 20:
                    self.ctrl.right(20)
                    time.sleep(4)
                elif self.States_Dict['y'] > y + 20:
                    self.ctrl.left(20)
                    time.sleep(4)
            elif y - 20 <= self.States_Dict['y'] < y - 5 or y + 20 >= self.States_Dict['y'] > y + 5:
                if self.States_Dict['y'] < y - 5:
                    self.ctrl.right(30)
                    time.sleep(4)
                    self.ctrl.left(20)
                    time.sleep(4)
                elif self.States_Dict['y'] > y + 5:
                    self.ctrl.left(30)
                    time.sleep(4)
                    self.ctrl.right(20)
                    time.sleep(4)
            else:
                time.sleep(2)
                self.ctrl.stop()
                state_conf += 1
                print("stop")
            self.States_Dict = parse_state()
            showimg()
            if self.States_Dict['mid'] < 0:
                self.now_stage = self.taskstages.finding_location
                return
                # break
        print("Success")
        if self.now_stage == self.taskstages.finding_fire:
            self.now_stage = self.taskstages.passing_fire
        else:
            if self.passed_fire:
                self.now_stage = self.taskstages.finding_ball
            else:
                self.now_stage = self.taskstages.finding_fire

    def pass_fire(self):
        assert (self.now_stage == self.taskstages.passing_fire)
        self.fire_code = -1
        self.ctrl.forward(100)
        time.sleep(10)
        self.now_stage = self.taskstages.finding_location
        self.passed_fire = True


if __name__ == '__main__':
    rospy.init_node('tello_control', anonymous=True)

    control_pub = rospy.Publisher('command', String, queue_size=1)
    ctrl = control_handler(control_pub)
    infouper = info_updater()
    tasker = task_handle(ctrl)

    time.sleep(5.2)
    ctrl.takeoff()
    time.sleep(4)
    ctrl.up(60)
    time.sleep(4)

    tasker.main()

    # ctrl.land()
