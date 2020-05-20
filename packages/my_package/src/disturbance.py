#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, LanePose
import numpy as np
import time
import random as rdm

class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
        # construct publisher
        self.pub_wheels_cmd = self.publisher(str(os.environ['VEHICLE_NAME'])+"/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        rospy.on_shutdown(self.custom_shutdown)

        self.vref = 0.23
        self.L = 0.05



    def run(self):
        # publish message every 1/x second
        car_cmd_msg = WheelsCmdStamped()
        tnew = time.time()
        rdm.seed()

        #parameter to change amplitude of disturbance
        gain = 15.0

        while not rospy.is_shutdown():

            randomrate = rdm.random()
            randomrate = randomrate*4+0.5
            rate = rospy.Rate(randomrate) 

            omegarand = gain*(rdm.random()-0.5)

            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref + self.L * omegarand
            car_cmd_msg.vel_right = self.vref - self.L * omegarand

            self.pub_wheels_cmd.publish(car_cmd_msg)

            msg1 = omegarand
            rospy.loginfo("omegarand: %s" % msg1)

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


    
if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
