#!/usr/bin/env python
# -- coding: UTF-8 --
import cv2
from pyzbar.pyzbar import decode
import numpy as np
import os

ROWS = 600
COLS = 600


def IsQrRate(rate):
    # rate 是 np.array
    return np.logical_and(rate > 0.15, rate < 1.9)


# 横向黑白比例判断
def IsQrColorRateX(image, flag):
    # 默认处理的是二值化之后的图
    nr = int(image.shape[0] / 2)
    nc = image.shape[1]
    bar = image[nr, :]
    edge = np.where(bar[:-1] != bar[1:])[0]
    edge = np.array([-1] + list(edge) + [nc])
    count = edge[1:] - edge[:-1]  # 每个区域的长度
    if len(count) < 5:  # 至少有5个区域，考虑到切割图象边界处可能会出现7个区域
        return False

    # 横向黑白比例1:1:3:1:1
    maxCount = np.max(count)
    maxIdx = np.argmax(count)
    if maxIdx < 2 or maxIdx > len(count) - 3:
        return False
    rate = np.concatenate([count[maxIdx - 2:maxIdx], count[maxIdx + 1:maxIdx + 3]], axis=0).astype(
        np.float32) / maxCount
    rate = IsQrRate(rate)

    return False not in rate


# 纵向黑白比例判断
def IsQrColorRateY(image, flag):
    # 默认处理的是二值化之后的图
    nc = int(image.shape[1] / 2)
    nr = image.shape[0]
    bar = image[:, nc]
    edge = np.where(bar[:-1] != bar[1:])[0]
    edge = np.array([-1] + list(edge) + [nr])
    count = edge[1:] - edge[:-1]  # 每个区域的长度
    if len(count) < 5:  # 至少有5个区域，考虑到切割图象边界处可能会出现7个区域
        return False

    # 纵向黑白比例1:1:3:1:1
    maxCount = np.max(count)
    maxIdx = np.argmax(count)
    if maxIdx < 2 or maxIdx > len(count) - 3:
        return False
    rate = np.concatenate([count[maxIdx - 2:maxIdx], count[maxIdx + 1:maxIdx + 3]], axis=0).astype(
        np.float32) / maxCount
    rate = IsQrRate(rate)

    return False not in rate


# 判断是横纵两个方向比例
def IsQrColorRate(image, flag):
    x = IsQrColorRateX(image, flag)
    if not x:
        return False
    y = IsQrColorRateY(image, flag)
    return y


# 二维码切割
def CropImage(img, rotatedRect):
    center, size, angle = rotatedRect
    center, size = tuple(map(int, center)), tuple(map(int, size))
    rows, cols = img.shape[0], img.shape[1]
    M = cv2.getRotationMatrix2D(center, angle, 1)
    img_rot = cv2.warpAffine(img, M, (cols, rows))
    out = cv2.getRectSubPix(img_rot, size, center)
    _, img_crop = cv2.threshold(out, 100, 255, cv2.THRESH_BINARY)
    return img_crop


# 判断是否为二维码定位角
def IsQrPoint(contour, img, i):
    rotatedRect = cv2.minAreaRect(contour)
    # 最小大小限定
    if rotatedRect[1][0] < 10 or rotatedRect[1][1] < 10:
        return False
    # 将二维码从图中抠出来
    imgCrop = CropImage(img, rotatedRect)
    flag = i
    # 黑白比例1:1:3:1:1
    result = IsQrColorRate(imgCrop, flag)
    return result


# 二维码解码
def QrDecode(img):
    return decode(img)[0].data


def FindQr(imgSrc):
    qrPoint = []

    # 彩色图转灰度图
    imgGray = cv2.cvtColor(imgSrc, cv2.COLOR_BGR2GRAY)
    cv2.imshow("gray", imgGray)

    # 高斯平滑滤波
    img = cv2.GaussianBlur(imgGray, (5, 5), 0)
    cv2.imshow("gaussian", img)

    # 腐蚀
    erosion = cv2.morphologyEx(img, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    # erosion = cv2.erode(img, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    cv2.imshow("erode", erosion)

    # 二值化
    _, imgBinary = cv2.threshold(erosion, 100, 255, cv2.THRESH_BINARY)
    cv2.imshow("g", imgBinary)

    # 查找轮廓
    contours, hierarchy = cv2.findContours(imgBinary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    # 筛选定位角：黑色定位角满足父轮廓有两个子轮廓
    parentIdx = -1
    imgSrcClone = np.copy(imgSrc)  # 调试用
    for i in range(len(contours)):
        k = i
        ic = 0
        parentIdx = i
        # 计算子轮廓深度
        while hierarchy[0][k][2] != -1:
            k = hierarchy[0][k][2]
            ic = ic + 1

        # 判断是否为定位角
        if ic >= 2:
            isQr = IsQrPoint(contours[parentIdx], imgBinary, parentIdx)
            print(parentIdx, isQr)
            if isQr:
                qrPoint.append(contours[parentIdx])

        x, y, w, h = cv2.boundingRect(contours[i])
        cv2.rectangle(imgSrcClone, (x, y), (x + w, y + h), (255, 0, 0), 2)

    cv2.imshow("contour", imgSrcClone)

    # 根据相邻三个定位角确定二维码整体位置框
    qrCenter = []
    state = [0] * len(qrPoint)
    qrBox = []
    qrCode = []

    # 绘制二维码定位角并计算定位角质心
    for point in qrPoint:
        x, y, w, h = cv2.boundingRect(point)
        cv2.rectangle(imgSrc, (x, y), (x + w, y + h), (255, 0, 0), 2)
        center = np.array([x + w / 2.0, y + h / 2.0])
        qrCenter.append(center)

    # 判断是否构成一个二维码
    cnt = 1
    for i in range(len(qrPoint)):
        if state[i]:
            continue
        for j in range(len(qrPoint)):
            if j == i or state[j]:
                continue
            for k in range(len(qrPoint)):
                if k == j or k == i or state[k]:
                    continue
                Dij = np.sum((qrCenter[i] - qrCenter[j]) ** 2).astype(np.float32)
                Dik = np.sum((qrCenter[i] - qrCenter[k]) ** 2).astype(np.float32)
                Djk = np.sum((qrCenter[k] - qrCenter[j]) ** 2).astype(np.float32)
                ratio = Dij / Dik
                ratio1 = (Dij + Dik) / Djk
                if ratio > 0.6 and ratio < 1.6 and ratio1 > 0.85 and ratio1 < 1.15:
                    state[i] = 1
                    state[j] = 1
                    state[k] = 1
                    contour = np.concatenate([qrPoint[i], qrPoint[j], qrPoint[k]], axis=0)
                    rotatedRect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rotatedRect)
                    qrBox.append(np.int0(box))
                    imgTemp = CropImage(imgGray, rotatedRect)
                    # 图像显示裁剪后的二维码
                    cv2.imshow("crop" + str(cnt), imgTemp)
                    cnt += 1
                    # 解码二维码
                    qrCode.append(QrDecode(imgTemp))
    # 在原图上画出二维码区域，显示出来
    for box in qrBox:
        cv2.drawContours(imgSrc, [box], 0, (255, 0, 0), 2)
    cv2.imshow("QR", imgSrc)
    cv2.waitKey(0)

    return qrCode


if __name__ == '__main__':
    # 读入图像
    data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "experiment2", "imgs")
    image = cv2.imread(os.path.join(data_path, str(12) + ".jpg"))
    # 打印信息
    print(image.shape[:2])

    # 处理图像
    qrcode = FindQr(image)
    print("Detected QR Code:")
    print(qrcode)
