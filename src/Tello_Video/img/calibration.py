#-*- coding:utf-8 -*-

import cv2
import glob
import numpy as np

cbrow = 8
cbcol = 11

objp = np.zeros((cbrow*cbcol,3), np.float32)
objp[:,:2] = np.mgrid[0:cbcol,0:cbrow].T.reshape(-1,2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob("*.jpg")
for fname in images:
#对每张图片，识别出角点，记录世界物体坐标和图像坐标
    img = cv2.imread(fname) #source image
    #img = cv2.resize(img,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #转灰度
    #cv2.imshow('img',gray)
    #cv2.waitKey(0)
    #寻找角点，存入corners，ret是找到角点的flag
    ret, corners = cv2.findChessboardCorners(gray,(11,8),None)
    #print ret,corners
    #criteria:角点精准化迭代过程的终止条件
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    #执行亚像素级角点检测
    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    objpoints.append(objp)
    #print objpoints
    imgpoints.append(corners2)
    #在棋盘上绘制角点,只是可视化工具
    img = cv2.drawChessboardCorners(gray,(cbcol,cbrow),corners2,ret)
    cv2.imshow('img',img)
    cv2.waitKey(1000)
'''
传入所有图片各自角点的三维、二维坐标，相机标定。
每张图片都有自己的旋转和平移矩阵，但是相机内参和畸变系数只有一组。
mtx，相机内参；dist，畸变系数；revcs，旋转矩阵；tvecs，平移矩阵。
'''
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
#标定好的内参
print mtx,dist


#图像去畸变过程
img = cv2.imread('over.jpg')
#img = cv2.resize(img,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
h,w = img.shape[:2]
'''
优化相机内参（camera matrix），这一步可选。
参数1表示保留所有像素点，同时可能引入黑色像素，
设为0表示尽可能裁剪不想要的像素，这是个scale，0-1都可以取。
'''
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
#纠正畸变
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

#这步只是输出纠正畸变以后的图片
x,y,w,h = roi
print roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibresult.png',dst)
#打印我们要求的两个矩阵参数
print "newcameramtx:\n",newcameramtx
print "dist:\n",dist

