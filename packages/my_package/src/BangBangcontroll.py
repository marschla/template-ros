#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, LanePose
import numpy as np
import time

class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
        # construct publisher
        self.pub_wheels_cmd = self.publisher(str(os.environ['VEHICLE_NAME'])+"/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        #subscriber
        self.sub_pose = rospy.Subscriber(str(os.environ['VEHICLE_NAME'])+"/lane_filter_node/lane_pose", LanePose, self.control, queue_size=1)
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)
        #def. variables
        self.vdiff = 0.0
        self.vref = 0.23
        self.dist = 0.0
        self.arr = np.array([0.0,0.0,0.0])

    def filter(self,data):
        val = np.median(data)
        return val 

    def getvdiff(self,dist):

        #array for moving average filter
        for i in range(1,3):
            self.arr[i-1]=self.arr[i]
        self.arr[2] = dist

        derr = self.filter(self.arr)

        vdiff = 0.0

        if self.dist < 0.0:
            vdiff = -0.05
        if self.dist > 0.0:
            vdiff = 0.05

        message = vdiff
        rospy.loginfo('vdiff2: %s' % message)

        return vdiff

    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        car_cmd_msg = WheelsCmdStamped()
        tnew = time.time()
        while not rospy.is_shutdown():

            self.vdiff = self.getvdiff(self.dist)

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref + self.vdiff
            car_cmd_msg.vel_right = self.vref - self.vdiff

            self.pub_wheels_cmd.publish(car_cmd_msg)

            #printing messages to verfy that program is working
            message1 = self.dist
            message2 = self.vdiff
            rospy.loginfo('dist: %s' % message1)
            rospy.loginfo('vdiff1: %s' % message2)
            rate.sleep()

        #shutdown procedure, stopping motor movement etc.
    def custom_shutdown(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.get_rostime()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0
        self.pub_wheels_cmd.publish(stop_msg)
        rospy.sleep(0.5)
        rospy.loginfo("Shutdown complete oder?")


    def control(self,pose):
        self.dist = pose.d
        message = pose.d
        #rospy.loginfo('d =  %s' % message)

    
if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
