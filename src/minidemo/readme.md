# 本次比赛的小demo，大家可以阅读相应程序，供同学们参考
运行两个py文件即可完成简单的demo测试：rosrun demo tello_ros_demo.py和rosrun demo minicontrol.py

和之前让同学们直接在tello_ros下开发程序不同的是，minicontrol采用了ros通讯的机制完成整个的控制过程，大家可以参考这种实现方法，也可以使用之前的版本完成开发。

tello_ros_demo.py是上次课程上的程序，主要完成三件事情：publish一个图像信息，publish一个状态信息，subscribe一个command的控制信息

minicontrol.py是助教开发的控制程序，订阅图像和状态信息，通过publish到command话题来完成反馈控制，实现识别定位毯并且飞到定位毯的中心位置。

大家可以参考使用这种方式开发完成比赛的完整demo。

## Prerequisites

- Python2.7
- pip
- Python OpenCV
- Numpy 





