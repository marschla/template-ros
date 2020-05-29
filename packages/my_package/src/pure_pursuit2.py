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

        self.vref = 0.2
        self.L = 0.05
        self.omega = 0.0
        self.tnew = 0.0

        
        # Subscriptions
        self.sub_seg = rospy.Subscriber(str(os.environ['VEHICLE_NAME'])+"/ground_projection/lineseglist_out", SegmentList, self.process_segments, queue_size=1)
        #self.sub_pose = rospy.Subscriber(str(os.environ['VEHICLE_NAME'])+"/lane_filter_node/lane_pose", LanePose ,self.updatepose ,queue_size = 1)
        
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

    def LLS(self,arr_x,arr_y):
        B = np.matrix(arr_y).transpose()
        AT = np.matrix([np.ones(np.size(arr_x)),arr_x])
        A = AT.transpose()

        x = np.linalg.inv(AT*A)*AT*B
        return np.array([x[0,0],x[1,0]])

    def gettarget(self,x,d):
        a = 1 + x[1]**2
        b = 2*x[1]*x[0]
        c = x[0]**2-d**2

        x1 = (-b + np.sqrt(b**2-4*a*c))/(2*a)
        x2 = (-b - np.sqrt(b**2-4*a*c))/(2*a)

        if x1 > x2:
            x_tar = x1
        else:
            x_tar = x2

        y_tar = x[0] + x[1]*x_tar

        return np.array([x_tar,y_tar])

        

    def process_segments(self, input_segment_list):

        dmax = 0.4

        lookahead = 0.2

        yellow_arr_x = []
        yellow_arr_y = []
        white_arr_x = []
        white_arr_y = []
        
        for segment in input_segment_list.segments:
            point0 = segment.points[0]
            point1 = segment.points[1]

            ave_point_x = (point0.x + point1.x)/2.0
            ave_point_y = (point0.y + point1.y)/2.0

            d = np.sqrt(ave_point_x**2 + ave_point_y**2)

            if d > dmax:
                continue

            if segment.color == 1:
                yellow_arr_x.append(ave_point_x)
                yellow_arr_y.append(ave_point_y)

            if segment.color == 0:
                white_arr_x.append(ave_point_x)
                white_arr_y.append(ave_point_y)

        #perform LLS to get a linear func for white segs and for yellow segs

        flag = 0

        #yellow
        try:
            x_yellow = self.LLS(yellow_arr_y,yellow_arr_x)
        except:
            flag += 1

        #white
        try:
            x_white = self.LLS(white_arr_y,white_arr_x)
        except:
            flag += 2


        if flag == 0:
            x_ave = (x_yellow + x_white)/2.0
        elif flag == 1:
            x_ave = x_white + np.array([0.2,0.])
        elif flag == 2:
            x_ave = x_yellow + np.array([-0.15,0.])
        elif flag == 3:
            x_ave = np.array([0.,1.])
        else:
            rospy.loginfo("Programm should never end up here")

        target = self.gettarget(x_ave,lookahead)

        alpha = np.arctan2(target[1],target[0])
        self.omega = 4.0*self.vref* np.sin(alpha)/lookahead

        rospy.loginfo("target: %s" % x_ave)

        

    
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
