#!/usr/bin/env python
# -*- coding: utf-8 -*-

import shutil,os,time

os.system('rostopic pub -1 /command std_msgs/String \"takeoff\"')
time.sleep(3.1)
os.system('rostopic pub -1 /command std_msgs/String \"go 0 50 0 10\"')
time.sleep(3.1)
os.system('rostopic pub -1 /command std_msgs/String \"land\"')
