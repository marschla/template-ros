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
        #self.sub_lane_reading = rospy.Subscriber("/autobot17/lane_filter_node/seglist_filtered", SegmentList, self.process_segments, queue_size=1)
        self.sub_pose = rospy.Subscriber(str(os.environ['VEHICLE_NAME'])+"/line_detector_node/segment_list", SegmentList, self.process_segments, queue_size=1)

        # Publication
        #self.pub_car_cmd = rospy.Publisher("/{}/joy_mapper_node/car_cmd".format(location), Twist2DStamped, queue_size=1)
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

        num_white = 0
        num_yellow = 0
        num_white_far = 0
        num_yellow_far = 0
        '''
        total_white_x = 0
        total_white_y = 0
        total_yellow_x = 0
        total_yellow_y = 0
        total_white_far_x = 0
        total_white_far_y = 0
        total_yellow_far_x = 0
        total_yellow_far_y = 0
        '''

        total_white = np.zeros(2)
        total_white_far = np.zeros(2)
        total_yellow = np.zeros(2)
        total_yellow_far = np.zeros(2)

        far = 0.5

        for segment in all_segments:

            point0 = segment.points[0]
            point1 = segment.points[1]
            col = segment.color

            # We only care about the average point of the segment
            ave_point = np.array([point0.x + point1.x, point0.y + point1.y]) / 2.
            # ^ Q: is this misguided? Will this unfairly remove points?

            if col == 0:
                num_white += 1
                total_white += ave_point

                if ave_point.dot(ave_point) > far:
                    num_white_far += 1
                    total_white_far += ave_point

            elif col == 1:
                num_yellow += 1
                total_yellow += ave_point

                if ave_point.dot(ave_point) > far:
                    num_white_far += 1
                    total_yellow_far += ave_point

        #ave_white_far = total_white_far * 1. / num_white_far
        #ave_yellow_far = total_yellow_far * 1. / num_yellow_far

        # Tell the car to do based on what we see
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0

        if num_white + num_yellow == 0:
            # Want to turn right (improve later)
            car_control_msg.v = 0.2/2.0
            car_control_msg.omega = -3.0

        elif num_white == 0: # and num_yellow != 0
            if num_yellow_far > 0:
                ave_yellow = total_yellow_far * 1. / num_yellow_far
                ave_yellow[1] -= 0.15 # subtract offset
            else:
                ave_yellow = total_yellow * 1. / num_yellow
                ave_yellow[1] -= 0.25 # subtract offset

            alpha = np.arctan2(ave_yellow[1], ave_yellow[0])
            omega = 6 * np.sin(alpha)

            car_control_msg.v = 0.25/2.0
            car_control_msg.omega = omega

        elif num_yellow == 0: # and num_white != 0
            if num_white_far > 0:
                ave_white = total_white_far * 1. / num_white_far
                ave_white += 0.15 # add offset
            else:
                ave_white = total_white * 1. / num_white
                ave_white += 0.25 # add offset

            alpha = np.arctan2(ave_white[1], ave_white[0])
            omega = 6 * np.sin(alpha)

            car_control_msg.v = 0.25/2.0
            car_control_msg.omega = omega

        else: # see both colours
            if num_white_far > 0 and num_yellow_far > 0:
                ave_white = total_white_far * 1. / num_white_far
                ave_yellow = total_yellow_far * 1. / num_yellow_far
            else:
                ave_white = total_white * 1. / num_white
                ave_yellow = total_yellow * 1. / num_yellow

            overall_ave = (ave_white + ave_yellow) / 2.

            alpha = np.arctan2(overall_ave[1], overall_ave[0])
            omega = 3 * np.sin(alpha)

            car_control_msg.v = 0.4/2.0
            car_control_msg.omega = omega

        self.vref = car_control_msg.v
        self.omega = car_control_msg.omega

        

    
    def run(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            #rospy.loginfo("Hallo\n")

            car_cmd_msg = WheelsCmdStamped()

            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref - self.L * self.omega
            car_cmd_msg.vel_right = self.vref + self.L * self.omega

            msg1 = self.omega
            msg2 = self.vref

            rospy.loginfo("omega: %s" % msg1)
            rospy.loginfo("vref: %s" % msg2)

            # Send the command to the car
            #self.pub_car_cmd.publish(car_control_msg)
            self.pub_wheels_cmd.publish(car_cmd_msg)
            
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
