#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy
import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl


from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.msg import ModelStates
from turtlebot_junwoo.msg     import *
from tf.transformations  import euler_from_quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

class Camera:
    def __init__(self):

        self.pub = rospy.Publisher('/marker_position', Float32MultiArray, queue_size=10)
        self.bridge = CvBridge()

        self.pub_msg = Float32MultiArray()
        self.pub_msg.data = []


    def image_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        #rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, 0.5, )

        if ids:
            self.pub_msg.data = [0,0]
            self.pub_msg.data[0] = (corners[0][0,0,0] + corners[0][0,1,0] + corners[0][0,2,0] + corners[0][0,3,0] )/4
            self.pub_msg.data[1] = (corners[0][0,0,1] + corners[0][0,1,1] + corners[0][0,2,1] + corners[0][0,3,1]) /4
        else:
            self.pub_msg.data =[]

    def pub_position(self):
        # print('a')
        self.pub.publish(self.pub_msg) 



        #print(self.x, self.y, self.yaw, dsrd_yaw, yaw_err, self.pub_msg.angular.z)


def main():
    camera = Camera()
    rospy.init_node('marker_detect', anonymous=True)
    rospy.Subscriber('/Top/camera1/image_raw',   Image,  camera.image_cb)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        camera.pub_position()
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
