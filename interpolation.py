# -*- coding: UTF-8 -*-
import sys
import re
import numpy as np
from scipy import interpolate
import cv2


def read_image(name):
    """
    读取图像

    Arguments:
        name {string} -- [文件名]

    Returns:
        [np.array] -- 图像
    """
    fs = cv2.FileStorage(name, cv2.FileStorage_READ)
    image = fs.getNode('image').mat()
    fs.release()
    return image


def read_file(name):
    """
    读取CSV文件

    Arguments:
        name {string} -- [文件名]

    Returns:
         [array, array] -- [X坐标集合, Y坐标集合]
    """
    file = open(name, 'r')
    x = []
    y = []
    try:
        while True:
            line = file.readline()
            if line:
                array = re.split('[,\n]', line)
                x.append(float(array[0]))
                y.append(float(array[1]))
            else:
                break
    finally:
        file.close()

    return (np.array(x), np.array(y))


def write_file(name, x, y):
    """
    写入CSV文件

    Arguments:
        name {string} -- [文件名]
        x {array} -- [X坐标集合]
        y {array} -- [Y坐标集合]
    """
    file = open(name, 'w')
    try:
        for i in range(len(x)):
            file.write('%f, %f\n' % (x[i], y[i]))
    finally:
        file.close()


def compute_bspline(x, y, s=200):
    """
    计算B样条曲线

    Arguments:
        x {array} -- [X坐标集合]
        y {array} -- [Y坐标集合]

    Returns:
        [array, array] -- [X坐标集合, Y坐标集合]
    """
    tck, u = interpolate.splprep([x, y], s=s, per=1)
    u_new = np.linspace(u.min(), u.max(), num=(x.max() - x.min()))
    return interpolate.splev(u_new, tck, der=0)


def compute_bspline_derivatives(x, y, s=200):
    """
    计算B样条曲线一阶导数

    Arguments:
        x {array} -- [X坐标集合]
        y {array} -- [Y坐标集合]

    Returns:
        [array, array] -- [X坐标集合, Y坐标集合]
    """
    tck, u = interpolate.splprep([x, y], s=s, per=1)
    u_new = np.linspace(u.min(), u.max(), num=(x.max() - x.min()))
    return interpolate.splev(u_new, tck, der=1)


def compute_vector_by_k(x, y):
    """
    根据斜率计算单位向量

    Arguments:
        x {float} -- 横坐标
        y {float} -- 纵坐标

    Returns:
        [np.array] -- 单位向量
    """
    if abs(x) < sys.float_info.epsilon:
        v = np.array([1, 0])
        return v / np.linalg.norm(v)
    else:
        k = y / x
        v = np.array([1, k])
        return v / np.linalg.norm(v)


def compute_cross_vector_by_k(x, y):
    """
    根据斜率计算垂线单位向量

    Arguments:
        x {float} -- 横坐标
        y {float} -- 纵坐标

    Returns:
        [np.array] -- 单位向量
    """
    return compute_vector_by_k(-y, x)


def generate_contour(x, y):
    """
    生成轮廓

    Arguments:
        x {array} -- [X坐标集合]
        y {array} -- [Y坐标集合]

    Returns:
        [np.array] -- 轮廓
    """
    contour = []
    for i in range(len(x)):
        contour.append(np.array([x[i], y[i]], dtype='int'))
    return np.array(contour)


def compute_inlier_contour(x, y, distance):
    """
    计算内部点轮廓
    等距缩进版

    Arguments:
        x {array} -- [X坐标集合]
        y {array} -- [Y坐标集合]
        distance {float} -- [距离]

    Returns:
        [array, array] -- [X坐标集合, Y坐标集合]
    """
    x_new, y_new = compute_bspline(x, y)
    contour = generate_contour(x_new, y_new)
    x_der, y_der = compute_bspline_derivatives(x, y)
    x_inlier = []
    y_inlier = []
    for i in range(len(x_new)):
        v = compute_cross_vector_by_k(x_der[i], y_der[i]) * distance
        x = x_new[i] + v[0]
        y = y_new[i] + v[1]
        # 判断新点是否在轮廓外部
        if cv2.pointPolygonTest(contour, (x, y), False) < 0:
            x = x_new[i] - v[0]
            y = y_new[i] - v[1]
        x_inlier.append(int(x))
        y_inlier.append(int(y))

    return (x_inlier, y_inlier)


def compute_inlier_contour2(x, y, min, max, image):
    """
    计算内部点轮廓
    深度图最小值版

    Arguments:
        x {array} -- [X坐标集合]
        y {array} -- [Y坐标集合]
        min {int} -- [最小距离]
        max {int} -- [最大距离]
        image {np.array} -- [深度图]

    Returns:
        [array, array] -- [X坐标集合, Y坐标集合]
    """
    x_new, y_new = compute_bspline(x, y)
    contour = generate_contour(x_new, y_new)
    x_der, y_der = compute_bspline_derivatives(x, y)
    x_inlier = []
    y_inlier = []
    for i in range(len(x_new)):
        xMin = None
        yMin = None
        zMin = None
        v0 = compute_cross_vector_by_k(x_der[i], y_der[i])
        for d in range(min, max):
            v = v0 * d
            x = x_new[i] + v[0]
            y = y_new[i] + v[1]
            # 判断新点是否在轮廓外部
            if cv2.pointPolygonTest(contour, (x, y), False) < 0:
                x = x_new[i] - v[0]
                y = y_new[i] - v[1]

            z = abs(image[int(y), int(x)])
            if z < sys.float_info.epsilon:
                continue

            if zMin is None or z < zMin:
                xMin = x
                yMin = y
                zMin = z

        if zMin is not None:
            x_inlier.append(int(xMin))
            y_inlier.append(int(yMin))

    return (x_inlier, y_inlier)


def compute_inlier_contour3(x, y, min, max, deviation, image):
    """
    计算内部点轮廓
    K值稳定版

    Arguments:
        x {array} -- [X坐标集合]
        y {array} -- [Y坐标集合]
        min {int} -- [最小距离]
        max {int} -- [最大距离]
        deviation {float} -- [容忍误差]
        image {np.array} -- [K图]

    Returns:
        [array, array, array, array] -- [内部点X坐标集合, 内部点Y坐标集合, 中间点X坐标集合, 中间点Y坐标集合]
    """
    x_new, y_new = compute_bspline(x, y)
    contour = generate_contour(x_new, y_new)
    x_der, y_der = compute_bspline_derivatives(x, y)
    x_inlier = []
    y_inlier = []
    x_middle = []
    y_middle = []
    for i in range(len(x_new)):
        xMin = None
        yMin = None
        kMin = None
        v0 = compute_cross_vector_by_k(x_der[i], y_der[i])
        for d in range(min, max):
            v = v0 * d
            x = x_new[i] + v[0]
            y = y_new[i] + v[1]
            # 判断新点是否在轮廓外部
            if cv2.pointPolygonTest(contour, (x, y), False) < 0:
                x = x_new[i] - v[0]
                y = y_new[i] - v[1]

            k = abs(image[int(y), int(x)])
            if k < sys.float_info.epsilon:
                continue

            if kMin is None or kMin - k > deviation:
                xMin = x
                yMin = y
                kMin = k

        if kMin is not None:
            x_inlier.append(xMin)
            y_inlier.append(yMin)
            x_middle.append((x_new[i] + xMin) / 2.0)
            y_middle.append((y_new[i] + yMin) / 2.0)

    return (x_inlier, y_inlier, x_middle, y_middle)


def execute():
    image = read_image('./output/contours.yml')
    x, y = read_file('./output/contours.txt')
    x_new, y_new = compute_bspline(x, y)
    x_inlier, y_inlier, x_middle, y_middle = compute_inlier_contour3(
        x, y, 5, 30, 0.5, image)
    x_middle, y_middle = compute_bspline(
        np.array(x_middle), np.array(y_middle))
    write_file('./output/inlier_contours.txt', x_middle, y_middle)
