#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
import time

class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
        # construct publisher
        
    def run(self):
        # publish message every x second
        rate = rospy.Rate(5) 
        stoptime = 5.0
        t0 = time.time()
        
        
        while not rospy.is_shutdown():
            
            #stop programm once a certain time has passed (for experiments, not meant for normal usage)
            if (time.time()-t0)>stoptime:
                rospy.logwarn("Time's up!!!")
                rospy.signal_shutdown("Ende gut, alles gut")
            
            message5 = time.time()-t0
            #rospy.loginfo("time: %s" % message5)
            
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()

