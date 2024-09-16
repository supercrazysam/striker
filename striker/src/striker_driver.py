#!/usr/bin/env python3

###############################
# "Silver" ROS driver - joystick version
# Sam Shum (cshum@andrew.cmu.edu)  
# Copyright (c) 2023 Sam Shum
#
# DO NOT DISTRIBUTE / RE-DISTRIBUTE WITHOUT PERMISSION!
# This is a private code developed by Sam Shum, intended for Sam's private use under Ji Zhang's project,
# inform Sam Shum and give credit if you need to use this piece of code in your project.
#
# All rights reserved. 
###############################

import rospy
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import Joy

import time
import numpy as np

#######################
def clip(value, lower, upper):
    return lower if value < lower else upper if value > upper else value
    
    '''
    Wheel definition:
               |Forward|
               1        2
               3        4
               5        6
               |Backward|
    '''

class striker_joystick_handler(object):
    def __init__(self):
        node_name = "striker_joystick_handler"
        self.node_name = node_name
        rospy.init_node(node_name)
        rospy.loginfo("Starting node "+ str(node_name))
        rospy.on_shutdown(self.cleanup)

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback , queue_size=1)
        self.cmd_pub = rospy.Publisher("/chassis_cmd_vel",Twist, queue_size=1)  #TwistStamped
        self.rate = rospy.Rate(35) #30  #send out control command @20Hz

        self.last_command_time = rospy.Time.now()

        self.vx = self.vy = self.w = 0     
        
        self.vx_max = 2.0
        self.vy_max = 2.0
        self.w_max  = 4.0

        ############
        self.command = Twist()    #TwistStamped

    def joy_callback(self, data):
        #right joystick
        if data.axes[7]>0:   #for 8 channel
        #if data.axes[5]>0:   #0.7 = estop up (send command)   -0.7 = estop down (toggle safety estop, stop all action)
            
            self.vx = clip((data.axes[1] /  0.7) * self.vx_max,  -self.vx_max, self.vx_max)    # 0.707 forward max       -0.707 backward max
            self.vy = clip((data.axes[0] /  0.7) * self.vy_max,  -self.vy_max, self.vy_max)    # 0.707 forward max       -0.707 backward max
            self.w  = clip((data.axes[3] /  0.7) * self.w_max ,  -self.w_max , self.w_max )    # 0.707 left max         -0.707 right max
            self.last_command_time = rospy.Time.now()
        else:

            self.vx = 0
            self.vy = 0
            self.w  = 0
            self.last_command_time = rospy.Time.now()


               
        
        #self.last_command_time = rospy.Time.now()

    def cleanup(self):
        print("Shutting down striker driver")
        rospy.signal_shutdown("striker-driver shutdown")

    def mainloop(self):
        while not rospy.is_shutdown():
            
            if ((rospy.Time.now() - self.last_command_time).to_sec() > 1):
                self.command.linear.x      =  0
                self.command.linear.y      =  0
                self.command.angular.z     =  0
 
            else:
                
                self.command.linear.x   = self.vx
                self.command.linear.y   = self.vy
                self.command.angular.z  = self.w
                     
            self.cmd_pub.publish(self.command)            
            self.rate.sleep()


x=striker_joystick_handler()
x.mainloop()

    


