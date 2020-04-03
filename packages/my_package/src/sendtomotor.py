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
        self.pub_omega = self.publisher(str(os.environ['VEHICLE_NAME'])+"/kinematics_node/velocity", Twist2DStamped, queue_size=1)
        #subscriber
        self.sub_pose = rospy.Subscriber(str(os.environ['VEHICLE_NAME'])+"/lane_filter_node/lane_pose", LanePose, self.control, queue_size=1)
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)
        #def. variables
        self.vdiff = 0.0
        self.omega = 0.0
        self.vref = 0.23
        self.dist = 0.0
        self.dold = 0.0
        self.tist = 0.0

        #params used for PID control
        self.C_p = 0.0
        self.C_i = 0.0
        self.C_d = 0.0
        self.C_t = 0.0
        self.arr_d = np.array([0.0,0.0,0.0])
        self.arr_t = np.array([0.0,0.0,0.0])

        #structural paramters of duckiebot
        self.L = 0.05

    def filter(self,data):
        val = np.median(data)
        return val 

    def getomega(self,dist,tist,dt):
        #parameters for PID control
        k_p = 2.0
        k_i = 0.0
        k_d = 0.0
        k_t = 0.1
        sati = 0.05
        satd = 0.1
        satt = 0.05

        #array for moving average filter
        for i in range(1,3):
            self.arr_d[i-1]=self.arr_d[i]
        self.arr_d[2] = dist

        derr = self.filter(self.arr_d)

        for i in range(1,3):
            self.arr_t[i-1]=self.arr_t[i]
        self.arr_t[2] = tist

        terr = self.filter(self.arr_t)

        #proportional gain part
        self.C_p = k_p*derr

        #integral term
        self.C_i += dt*derr

        #make sure integral term doesnt become too big
        if self.C_i > sati:
            self.C_i = sati
        if self.C_i < -sati:
            self.C_i = -sati

        #derivative term
        self.C_d = k_d*(derr-self.dold)/dt
        self.dold = derr

        #make sure derivative term doesnt become too big
        if self.C_d > satd:
            self.C_d = satd
        if self.C_d < -satd:
            self.C_d = -satd

        #theta error part (currently only P-controlled), hopefully reduces overshoot and oscilation
        self.C_t = k_t*terr

        #saturation procedure for theta term
        if self.C_t > satt:
            self.C_t = satt 
        if self.C_t < -satt:
            self.C_t = -satt 


        #vdiff = self.C_p + k_i*self.C_i + self.C_d + self.C_t
        omega = self.C_p + k_i*self.C_i + self.C_d + self.C_t
        return omega

    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        car_cmd_msg = WheelsCmdStamped()
        tnew = time.time()
        while not rospy.is_shutdown():

            told = tnew
            tnew = time.time()
            dt = tnew-told

            #self.vdiff = self.getvdiff(self.dist,self.tist,dt)
            self.omega = self.getomega(self.dist,self.tist,dt)

            

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref + self.L * self.omega
            car_cmd_msg.vel_right = self.vref - self.L * self.omega

            self.pub_wheels_cmd.publish(car_cmd_msg)

            #printing messages to verfy that program is working
            message1 = self.dist
            message2 = self.omega
            message3 = self.tist
            rospy.loginfo('dist: %s' % message1)
            rospy.loginfo('vdif: %s' % message2)
            rospy.loginfo('tist: %s' % message3)
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
        self.tist = pose.phi
        #message = pose.d
        #rospy.loginfo('d =  %s' % message)

    
if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
