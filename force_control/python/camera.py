#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

import cv2
import numpy as np
from matplotlib import pyplot as plt

class image_processor:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/iiwa/camera1/image_raw", Image, self.callback)
        self.point_pub = rospy.Publisher("/iiwa/camera1/line_coordinate", Point, queue_size=2)

        self.x0 = 480
        self.y0 = 320

        self.point = Point(0, 0, 0)
        # 用于控制最终发布点的长度大小（相当于方向向量的幅值）
        self.vel = 1

    # 收到新的图像数据后被触发
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # 对图像进行裁剪，将原图像的底部100行和右侧200列去掉，并转换为灰度图
        gray = cv2.cvtColor(cv_image[:-100, :-200], cv2.COLOR_BGR2GRAY)
        # 对灰度图进行 Canny 边缘检测，阈值为(100, 200)
        edges = cv2.Canny(gray, 100, 200)
        # 获取边缘图像中所有白色像素点的坐标（行 列）
        white_poisnts = np.argwhere(edges == 255)
        p = (0, 0)
        x = [0, 0]
        norm = 1.0

        # 如果检测到有边缘点，则取 white_poisnts[-1] （最后一个点）作为线的末端点
        if white_poisnts.size > 0:
            p = (white_poisnts[-1, 1], white_poisnts[-1, 0])
            # 实际p = (x坐标, y坐标)，p[0]是x坐标（列方向），p[1]是y坐标（行方向）
            x = [p[1] - self.x0, -p[0] + self.y0]
            # norm是向量 x 的模，用于后面的归一化
            norm = np.sqrt(x[0]**2 + x[1]**2)

        # 二维图像场景下通常也不需要用到 z 轴信息
        self.point.x = self.vel*x[0]/norm
        self.point.y = self.vel*x[1]/norm
        self.point_pub.publish(self.point)
        # 在原图像上用一个红色小圆标记所选取的点 p
        cv_image = cv2.circle(cv_image, p, radius = 5, color=(0, 0, 255), thickness=-1)
        # # print(x)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('iiwa_camera_processor')
    processor = image_processor()
    rospy.loginfo("HI")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
