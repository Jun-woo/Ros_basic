#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy
import numpy as np


from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.msg import ModelStates
from algorithm.msg       import *
from tf.transformations  import euler_from_quaternion



class Turtlebot3:
    def __init__(self):
        self.x=0;   self.y=0; self.yaw=0;   
        self.turtlebot_i = -1
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.goal = [5,0]
        self.pub_msg = Twist()


    def modelstate_cb(self,data):

        if self.turtlebot_i == -1:
            for i in range(0,len(data.pose)):
                if data.name[i] =="turtlebot3_burger" : break
                self.turtlebot_i = i+1
        self.x = data.pose[self.turtlebot_i].position.x
        self.y = data.pose[self.turtlebot_i].position.y
        quat_x = data.pose[self.turtlebot_i].orientation.x
        quat_y = data.pose[self.turtlebot_i].orientation.y
        quat_z = data.pose[self.turtlebot_i].orientation.z
        quat_w = data.pose[self.turtlebot_i].orientation.w
        quaternion = (quat_x, quat_y, quat_z, quat_w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def pub_control(self):

        x_err = self.goal[0]-self.x
        y_err = self.goal[1]-self.y

        dist_err = np.sqrt(np.square(x_err)+np.square(y_err))
        dsrd_yaw = np.arctan2(y_err,x_err)

        yaw_err = dsrd_yaw-self.yaw

        if yaw_err <-2*np.pi:
            yaw_err = yaw_err + 2*np.pi

        if yaw_err > 2*np.pi:
            yaw_err = yaw_err - 2*np.pi

        self.pub_msg.linear.x = 0.1
        self.pub_msg.angular.z = 5*yaw_err

        if dist_err <1:
            self.pub_msg.linear.x = 0.0
            self.pub_msg.angular.z = 0.0    

        self.pub.publish(self.pub_msg)

        #print(self.x, self.y, self.yaw, dsrd_yaw, yaw_err, self.pub_msg.angular.z)


def main():

    rospy.init_node('turtlebot_control', anonymous=True)
    m_Turtlebot3 = Turtlebot3()
    rospy.Subscriber('/gazebo/model_states',          ModelStates,  m_Turtlebot3.modelstate_cb)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m_Turtlebot3.pub_control()
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
