#!/usr/bin/env python
# coding=utf-8
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import pdb
import os


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
    # 实验二：坐标转换
    colorImgs = []  # 保存RGB图的列表
    depthImgs = []  # 保存深度图的列表
    poses = []  # 保存相机的外参，即转换后的变换矩阵

    # 读取文件，将外参保存在列表中
    data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "experiment1")
    pose_data = np.loadtxt(os.path.join(data_path, "pose.txt"))

    for i in range(5):
        # 利用opencv读取RGB和深度图，按顺序保存在colorImgs和depthImgs中
        colorImgs.append(cv2.imread(os.path.join(data_path, "color", str(i + 1) + ".png")))
        depthImgs.append(cv2.imread(os.path.join(data_path, "depth", str(i + 1) + ".pgm"),  cv2.IMREAD_UNCHANGED))

        # 将poses列表中的单元转成变换矩阵T储存
        R = q2R(np.roll(pose_data[i][3:], 1))  # 将实部移至四元数开始以代入q2R函数
        t = pose_data[i][:3]
        poses.append(R2T(R, t))

    # 计算点云并拼接
    # 相机内参
    cx = 325.5
    cy = 253.5
    fx = 518.0
    fy = 519.0
    depthScale = 1000.0
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    # 新建一个列表，保存所有点的世界坐标
    cloudWorld = []

    for i in range(5):
        print("processing image " + str(i + 1))
        [rows, cols] = depthImgs[i].shape
        for r in range(rows):
            for c in range(cols):
                # color = colorImgs[i][r, c]
                depth = depthImgs[i][r, c]
                T = poses[i]
                Puv = np.array([c, r, 1])

                # 将像素坐标转换成相机坐标
                Pc = np.dot(np.linalg.inv(K), Puv) * depth / depthScale
                Pc = np.append(Pc, 1.0)

                # 利用变换矩阵T，将相机坐标转换成世界坐标
                Pw = np.dot(T, Pc)[:3]

                # 将世界坐标添加到cloudWorld中
                cloudWorld.append(Pw)

    # ROS相关操作，将点云信息发布出去
    rospy.init_node('test', anonymous=True)
    pub_cloud = rospy.Publisher("/points", PointCloud2)
    while not rospy.is_shutdown():
        pcloud = PointCloud2()
        pcloud = point_cloud2.create_cloud_xyz32(pcloud.header, cloudWorld)

        pcloud.header.frame_id = "/map"
        pub_cloud.publish(pcloud)
        # rospy.loginfo(pcloud)
        rospy.sleep(1.0)
