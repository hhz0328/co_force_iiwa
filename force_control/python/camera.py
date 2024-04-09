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
        self.vel = 1

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_image[:-100, :-200], cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray, 100, 200)
        white_poisnts = np.argwhere(edges == 255)
        p = (0, 0)
        x = [0, 0]
        norm = 1.0
        if white_poisnts.size > 0:
            p = (white_poisnts[-1, 1], white_poisnts[-1, 0])
            x = [p[1] - self.x0, -p[0] + self.y0]
            norm = np.sqrt(x[0]**2 + x[1]**2)

        self.point.x = self.vel*x[0]/norm
        self.point.y = self.vel*x[1]/norm
        self.point_pub.publish(self.point)

        cv_image = cv2.circle(cv_image, p, radius = 5, color=(0, 0, 255), thickness=-1)
        # # print(x)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('iiwa_camera_processor')
    processor = image_processor()
    rospy.loginfo("HI")

    # img = cv2.imread('image.png', 1)
    # print(img.shape)
    # x0 = img.shape[0]
    # y0 = img.shape[1]/2

    # edges = cv2.Canny(img[:-100, :], 100, 200)
    
    # white_poisnts = np.argwhere(edges == 255)
    # p = (white_poisnts[-1, 1], white_poisnts[-1, 0])
    # print(p)
    # x = [p[1] - x0, -p[0] + y0]
    # print(x)

    # image = cv2.circle(img, p, radius = 5, color=(255, 0, 0), thickness=-1)
    # plt.imshow(image)
    # plt.show()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
    # cv2.destroyAllWindows()
