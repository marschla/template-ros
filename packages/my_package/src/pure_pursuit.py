#!/usr/bin/env python

import os
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, SegmentList, Segment, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading
import math
import time
from duckietown import DTROS

class MyNode(DTROS):
    def __init__(self,node_name):

        super(MyNode, self).__init__(node_name=node_name)

        self.vref = 0.0
        self.L = 0.05
        self.omega = 0.0
        self.tnew = 0.0

        # Start rospy for this node
        #rospy.init_node("lane_controller_node", anonymous=False)

        # Subscriptions
        self.sub_pose = rospy.Subscriber(str(os.environ['VEHICLE_NAME'])+"/ground_projection/lineseglist_out", SegmentList, self.process_segments, queue_size=1)
        
        # Publication
        self.pub_wheels_cmd = self.publisher(str(os.environ['VEHICLE_NAME'])+"/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        # Stop on shutdown
        rospy.on_shutdown(self.custom_shutdown)



    def custom_shutdown(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.get_rostime()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0

        self.pub_wheels_cmd.publish(stop_msg)
        rospy.sleep(0.5)
        rospy.loginfo("Shutdown complete oder?")


    def process_segments(self, input_segment_list):
        all_segments = input_segment_list.segments # this is a list of type Segment

        far = 0.2
        tol = 0.05

        num_yellow = 0
        num_white = 0

        yellow_arr = np.zeros(2)
        white_arr = np.zeros(2)

        rospy.loginfo("Hallo1")

        flag = True

        for segment in all_segments:
            point0 = segment.points[0]
            point1 = segment.points[1]

            ave_point_x = (point0.x + point1.x)/2.0
            ave_point_y = (point0.y + point1.y)/2.0

            d = np.sqrt(ave_point_x**2 + ave_point_y**2)

            if segment.color == 1:    #yellow color 
                if d < far + tol and d > far - tol:
                    num_yellow += 1
                    yellow_arr += np.array([ave_point_x,ave_point_y])

            if segment.color == 0 and ave_point_y < 0.1:     #white color 
                if d < far + tol and d > far - tol:
                    num_white += 1
                    white_arr += np.array([ave_point_x,ave_point_y])

        if num_white != 0 and num_yellow != 0:
            ave_white = white_arr * 1. / num_white
            ave_yellow = yellow_arr * 1. / num_yellow

            ave_point = (ave_white + ave_yellow)/2.0

            self.vref = 0.2


        if num_white == 0 and num_yellow == 0:
            #no white/yellow segments detected 
            #figure a procedure, if camera doesn't pick up any segments
            self.vref = 0.1
            self.omega = 2.5
            flag = False

        if num_white == 0 and num_yellow != 0:
            #only yellow segments (probably too far to the left)

            ave_yellow = yellow_arr * 1. / num_yellow

            offset = -0.15
            ave_point = ave_yellow + np.array([0.0,offset])

            self.vref = 0.2

        if num_yellow == 0 and num_white != 0:
            #only white segments (probably too far to the right of the lane)

            ave_white = white_arr * 1. / num_white

            offset = 0.35
            ave_point = ave_white + np.array([0.0,offset])

            self.vref = 0.2


        #rospy.loginfo("Flag: %s" % flag)
        
        #rospy.loginfo("yellow: %s" % num_yellow)
        #rospy.loginfo("white %s" % num_white)

        if flag == True:
            alpha = np.arctan2(ave_point[1],ave_point[0])

            self.omega = 4.0*self.vref* np.sin(alpha)/far

            rospy.loginfo("target: %s" % ave_point)
            

    
    def run(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            #rospy.loginfo("Hallo\n")

            car_cmd_msg = WheelsCmdStamped()

            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref - self.L * self.omega
            car_cmd_msg.vel_right = self.vref + self.L * self.omega

            '''
            msg1 = self.omega
            msg2 = self.vref

            rospy.loginfo("omega: %s" % msg1)
            rospy.loginfo("vref: %s" % msg2)
            '''

            # Send the command to the car
            self.pub_wheels_cmd.publish(car_cmd_msg)
            
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node

    #rospy.loginfo("Hallo!?!")

    node.run()
    # keep spinning
    rospy.spin()
