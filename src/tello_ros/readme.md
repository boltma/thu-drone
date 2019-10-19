# Tello_ros

tello_ros可以发送state和image信息到相应话题,将tello_ros文件夹下载到catkin_ws/src/下

## Prerequisites

- Python2.7
- pip
- Python OpenCV
- Numpy 
- PIL
- libboost-python
- Tkinter
- Python h264 decoder
    - <https://github.com/DaWelter/h264decoder>

## Installation


- **Linux (Ubuntu 14.04 and above)**
安装依赖
1.sudo apt-get update -y
2.sudo apt-get install libboost-all-dev -y
3.sudo apt-get install libavcodec-dev -y
4.sudo apt-get install libswscale-dev -y
5.sudo apt-get install python-numpy -y
6.sudo apt-get install python-matplotlib -y
7.sudo pip install opencv-python
8.sudo apt-get install python-imaging-tk
有可能还需要安装一下python-libboost，ffmpeg
安装h264
cd h264decoder
mkdir build
cd build
cmake ..
make
cp libh264decoder.so ../../

## Run the project
- **Step1**. Turn on Tello and connect your computer device to Tello via wifi.


- **Step2**. Open project folder in terminal. Run:
    
    ```
    python tello_ros.py(第一次可能会报错，再运行一次)
    ```


## Project Description

### tello.py - class Tello

Wrapper class to interact with Tello drone.
Modified from <https://github.com/microlinux/tello>

The object starts 3 threads:

 1. thread for receiving command response from Tello 端口8888
 2. thread for receiving video stream 端口11111
 3. thread for receiving state 端口8890

You can use **read_frame()** to read the last frame from Tello camera, and pause the video by setting **video_freeze(is_freeze=True)**.
You can use **read_state()** to read the state of Tello**.


### h264decoder - class libh264decoder

From <https://github.com/DaWelter/h264decoder>.

A c++ based class that decodes raw h264 data. This module interacts with python language via python-libboost library, and its decoding functionality is based on ffmpeg library. 

After compilation, a libh264decoder.so or libh264decoder.pyd file will be placed in the working directory so that the main python file can reference it. 

If you have to compile it from source,with Linux or Mac,you can:

```

```


