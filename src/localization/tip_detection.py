#!/usr/bin/env python3

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy
import tf2_ros

# ROS message types we need to use
from sensor_msgs.msg import NavSatFix, Imu

# SE3 library for handling poses and TF tree
from util.SE3 import SE3

class tip_detection: 
    def __init__(self):
        rospy.Subscriber("imu/imu_only", Imu, self.detect_tip)
        #read the orientation data from the given Imu message, store that value in `self.pose`, then publish that pose to the TF tree.
    def imu_callback(self, msg: Imu):
        self.pose = SE3.from_pos_quat(self.pose.position, np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")
        #subscribe to IMU
    def detect_tip(self):
        
        
        # failure states:

         # tipping over (pitch):

        # getting stuck while turning (yaw)

        # rolling over (roll):
        

        # radius = 0
        
        # angle = 0
        # t0 = 0
        # t1 = 1
        # final_distance_point  = 0
        # initial_distance_point = 0
        # s = final_distance_point - initial_distance_point
        # or
        # s = radius * angle 

        # change_in_time = t1 - t0
        # change_in_angular_displacement = s / radius
        # w = change_in_angular_displacement/change_in_time