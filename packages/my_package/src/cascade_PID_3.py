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
        #self.pub_omega = self.publisher(str(os.environ['VEHICLE_NAME'])+"/kinematics_node/velocity", Twist2DStamped, queue_size=1)
        #subscriber
        self.sub_pose = rospy.Subscriber(str(os.environ['VEHICLE_NAME'])+"/lane_filter_node/lane_pose", LanePose, self.control, queue_size=1)
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)
        #def. variables
        self.omega = 0.0
        #self.vref = 0.23    #v_ref defines speed at which the robot moves 
        self.dist = 0.0
        self.dold = 0.0
        self.phi = 0.0

        #params used for PID control 
        self.C_p = 0.0
        self.C_i = 0.0
        self.C_d = 0.0
        self.arr_d = np.array([0.0,0.0,0.0])

        #structural paramters of duckiebot
        self.L = 0.05      #length from point A to wheels [m]


    #moving avarge filter data is a 3x1 array, which includes the past three error values
    def filter(self,data):
        val = np.median(data)
        return val 

    def getphiref(self,dist,phi):
        L = 0.01         #lookahead parameter
        vmax = 0.25     #maximal velocity

        p = False

        
        #figuring out if DB moves towards centerline at 
        if dist>0 and phi<0:
            p = True

        if dist<0 and phi>0:
            p = True
        
        #execute only if moving towards centerline
        if p==True:
            xdot = 1.0
            ydot = np.tan(phi)*np.exp(-L/dist*np.tan(phi))
            

        #execute if not moving towards centerline
        if p == False and dist != 0:
            xdot = 1
            nom = L-dist*np.tan(phi)
            denom = np.square(dist/np.cos(phi))-np.square(L-dist*np.tan(phi))
            ydot = nom/np.sqrt(denom)

        if p==False and dist == 0:
            ydot = 0
            xdot = 1
            
        
        phiref = np.arctan2(ydot,xdot)

        vref = 0.2
        '''
        msg1 = phiref
        msg2 = dist
        msg3 = phi
        rospy.loginfo("phiref: %s" %msg1)
        rospy.loginfo("dist: %s" %msg2)
        rospy.loginfo("phi: %s" %msg3)
        '''
        return phiref,vref

    def getomega(self,err,dt):
        #parameters for PID control
        k_p = 5.0
        k_i = 0.0
        k_d = 0.2
        #saturation params
        sati = 1.0
        satd = 2.0
        omegasat=5.0
        
        '''
        #array for moving average filter
        for i in range(1,3):
            self.arr_d[i-1]=self.arr_d[i]
        self.arr_d[2] = 6*dist+tist    #error is weighted sum of distance to lane center and heading error

        #getting the filtered error value
        derr = self.filter(self.arr_d)
        '''

        #proportional gain part
        self.C_p = k_p*err

        #integral term (approximate integral)
        self.C_i += k_i*dt*err
        
        #make sure integral term doesnt become too big
        if self.C_i > sati:
            self.C_i = sati
        if self.C_i < -sati:
            self.C_i = -sati
        
        #derivative term
        self.C_d = k_d*(err-self.dold)/dt
        self.dold = err
        
        #make sure derivative term doesnt become too big
        if self.C_d > satd:
            self.C_d = satd
        if self.C_d < -satd:
            self.C_d = -satd
        
        #computing control output
        omega = self.C_p + self.C_i + self.C_d
        
        
        if omega>omegasat:
            omega=omegasat
        if omega<-omegasat:
            omega=-omegasat
        
        return omega

    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(2) 
        car_cmd_msg = WheelsCmdStamped()
        tnew = time.time()
        while not rospy.is_shutdown():
            #computing dt for I-part of controller
            told = tnew
            tnew = time.time()
            dt = tnew-told

            phiref, vref = self.getphiref(self.dist,self.phi)

            err = phiref - self.phi

            self.omega = self.getomega(err,dt)

            

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = vref - self.L * self.omega
            car_cmd_msg.vel_right = vref + self.L * self.omega

            self.pub_wheels_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lan_pose
            message1 = self.dist
            message2 = self.omega
            message3 = phiref
            message4 = vref
            
            rospy.loginfo('d: %s' % message1)
            rospy.loginfo('phiref: %s' % message3)
            rospy.loginfo('vref: %s' % message4)
            rospy.loginfo('omega: %s' % message2)
            
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

    #function updates pose variables, that camera gives us data at higher rate then this code operates at,
    #thus we do not use all incoming data
    def control(self,pose):
        self.dist = pose.d
        self.phi = pose.phi
        #message = pose.d
        #rospy.loginfo('d =  %s' % message)

    
if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
