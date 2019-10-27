#!/usr/bin/env python
# coding=utf-8
import numpy as np
import cv2
import rospy
import os
import sys
import getopt
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2


# 实验一：三维空间刚体运动的描述方式


def q2R(q):
    """
    功能：四元数转旋转矩阵
    输入：q 1x4的四元数向量，实部在前，虚部在后
    返回值：R 3x3的旋转矩阵
    其中输入和返回值都用numpy结构
    """
    q = np.outer(q, q) * 2.0 / np.dot(q, q)
    return np.array(
        [[1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[0, 3], q[1, 3] + q[0, 2]],
         [q[1, 2] + q[0, 3], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[0, 1]],
         [q[1, 3] - q[0, 2], q[2, 3] + q[0, 1], 1.0 - q[1, 1] - q[2, 2]]])


def R2q(R):
    """
    功能：旋转矩阵转四元数
    输入：R 3x3的旋转矩阵
    返回值：q 1x4的四元数向量，实部在前，虚部在后
    其中输入和返回值都用numpy结构
    """
    r = np.sqrt(1 + np.trace(R)) / 2.0
    return np.array(
        [r,
         (R[1, 2] - R[2, 1]) / (4.0 * r),
         (R[2, 0] - R[0, 2]) / (4.0 * r),
         (R[0, 1] - R[1, 0]) / (4.0 * r)
         ]
    )


def R2T(R, t):
    """
    功能：旋转矩阵转变换矩阵
    输入：R 3x3的旋转矩阵 t 3维的平移向量
    返回值：T 4x4的变换矩阵
    其中输入和返回值都用numpy结构
    """
    return np.vstack((np.hstack((R, np.transpose([t]))), [0, 0, 0, 1]))


def T2R(T):
    """
    功能：变换矩阵转旋转矩阵
    输入：T 4x4的变换矩阵
    返回值：R 3x3的旋转矩阵
    其中输入和返回值都用numpy结构
    """
    return T[:3, :3]


if __name__ == '__main__':
    """
    参数输入：
    -p --path   数据文件夹相对路径，默认为'./experiment1'，内含color、depth文件夹及pose.txt
    -n --num    所选图片编号，范围1~5
    -r --row    所选像素行号
    -c --col    所选像素列号
    示例：python pointcloud.py -n 1 -r 43 -c 217 应输出[-3.23940916 -2.52866315  6.15110785]的坐标
    """
    data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "experiment1")
    n = -1
    r = -1
    c = -1

    opts, args = getopt.getopt(sys.argv[1:], '-p:-n:-r:-c:', ['path=', 'num=', 'row=', 'col='])
    for opt_name, opt_value in opts:
        if opt_name in ('-p', '--path'):
            data_path = opt_value
        if opt_name in ('-n', '--num'):
            n = int(opt_value)
        if opt_name in ('-r', '--row'):
            r = int(opt_value)
        if opt_name in ('-c', '--col'):
            c = int(opt_value)

    # 实验二：坐标转换
    colorImgs = []  # 保存RGB图的列表
    depthImgs = []  # 保存深度图的列表
    poses = []  # 保存相机的外参，即转换后的变换矩阵

    # 读取文件，将外参保存在列表中
    pose_data = np.loadtxt(os.path.join(data_path, "pose.txt"))

    for i in range(5):
        # 利用opencv读取RGB和深度图，按顺序保存在colorImgs和depthImgs中
        colorImgs.append(cv2.imread(os.path.join(data_path, "color", str(i + 1) + ".png")))
        depthImgs.append(cv2.imread(os.path.join(data_path, "depth", str(i + 1) + ".pgm"), cv2.IMREAD_UNCHANGED))

        # 将poses列表中的单元转成变换矩阵T储存
        R = q2R(np.roll(pose_data[i][3:], 1))  # 将实部移至四元数开始以代入q2R函数
        t = pose_data[i][:3]
        poses.append(R2T(R, t))

    flag = True
    if n - 1 not in range(5) or r not in range(colorImgs[n - 1].shape[0]) or c not in range(colorImgs[n - 1].shape[1]):
        print("Image number or pixel location not specified or out of range.")
        flag = False

    # 计算点云并拼接
    # 相机内参
    cx = 325.5
    cy = 253.5
    fx = 518.0
    fy = 519.0
    depthScale = 1000.0
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    K_inv = np.linalg.inv(K)

    # 新建一个列表，保存所有点的世界坐标
    cloudWorld = []

    # 保存所计算的指定点坐标
    coordinates = []

    for i in range(5):
        print("processing image " + str(i + 1))

        T = poses[i]
        color = colorImgs[i]
        depth = depthImgs[i]

        # 将纵坐标、横坐标叠于深度之上，形成向量化的像素坐标
        [rows, cols] = depth.shape
        x, y = np.meshgrid(range(cols), range(rows))
        Puv = np.dstack((x, y, np.ones(depth.shape))).reshape(-1, 3).T

        # 将像素坐标转换成相机坐标
        Pc = np.dot(K_inv, Puv) * depth.flatten() / depthScale
        Pc = np.vstack((Pc, np.ones(rows * cols)))  # 补1以形成齐次坐标

        # 利用变换矩阵T，将相机坐标转换成世界坐标
        Pw = np.dot(T, Pc)[:3]

        # 将世界坐标添加到cloudWorld中
        color = np.array(color.reshape(-1, 3).T, dtype=np.int32)  # 将原图转为二维
        rgb = (color[2] << 16) + (color[1] << 8) + color[0]  # 将rgb信息打包
        cloudWorld.extend(np.array(np.vstack((Pw, rgb)), dtype=np.float32).T)

        # 获取指定点的坐标
        if flag and n - 1 == i:
            coordinates = Pw[:, r * cols + c]

    if flag:
        print("The coordinates of the specified pixel is: " + str(coordinates))

    # ROS相关操作，将点云信息发布出去
    rospy.init_node('test', anonymous=True)
    pub_cloud = rospy.Publisher("/points", PointCloud2, queue_size=10)
    pcloud = PointCloud2()

    # make point cloud
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.UINT32, 1),
              ]
    pcloud = point_cloud2.create_cloud(pcloud.header, fields, cloudWorld)
    pcloud.header.frame_id = "/map"
    print("pointcloud created")

    while not rospy.is_shutdown():
        pub_cloud.publish(pcloud)
        # rospy.loginfo(pcloud)
        rospy.sleep(1.0)
