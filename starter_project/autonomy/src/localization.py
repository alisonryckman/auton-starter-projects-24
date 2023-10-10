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

import math


class Localization:
    pose: SE3

    def __init__(self):
        # create subscribers for GPS and IMU data, linking them to our callback functions
        # TODO
        rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/imu/imu_only", Imu, self.imu_callback)

        # create a transform broadcaster for publishing to the TF tree
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # initialize pose to all zeros
        self.pose = SE3()

    def gps_callback(self, msg: NavSatFix):
        # How do I grab this from the XML file

        # rospy.loginfo(self.pose.position[0] + "0")
        # rospy.loginfo(self.pose.position[1] + "1")
        # rospy.loginfo(self.pose.position[2] + "2")

        lat = msg.latitude
        lon = msg.longitude

        ref_lat = 42.293195
        ref_lon = -83.7096706
        sphere_coord = [lat, lon]
        reference_coord = [ref_lat, ref_lon]

        pos = Localization.spherical_to_cartesian(sphere_coord, reference_coord)
        newPose = SE3(pos, self.pose.rotation)
        self.pose = newPose
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")
        """
        This function will be called every time this node receives a NavSatFix message
        on the /gps topic. It should read the GPS location from the given NavSatFix message,
        convert it to cartesian coordinates, store that value in `self.pose`, then publish
        that pose to the TF tree.
        """
        # TODO

    def imu_callback(self, msg: Imu):
        newPose = SE3.from_pos_quat(
            self.pose.position,
            quaternion=np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]),
        )
        self.pose = newPose
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")
        """
        This function will be called every time this node receives an Imu message
        on the /imu topic. It should read the orientation data from the given Imu message,
        store that value in `self.pose`, then publish that pose to the TF tree.
        """
        # TODO

    @staticmethod
    def spherical_to_cartesian(spherical_coord: np.ndarray, reference_coord: np.ndarray) -> np.ndarray:
        circumference = 63710 * 2
        dLat = spherical_coord[0] - reference_coord[0]
        dLon = spherical_coord[1] - reference_coord[1]
        y = circumference * dLat
        x = circumference * dLon * np.cos(np.radians(reference_coord[0]))
        rospy.loginfo(x)
        rospy.loginfo("^X")

        rospy.loginfo(y)
        rospy.loginfo("^Y")

        z = 0
        return np.array([x, y, z])
        """
        This is a utility function that should convert spherical (latitude, longitude)
        coordinates into cartesian (x, y, z) coordinates using the specified reference point
        as the center of the tangent plane used for approximation.
        :param spherical_coord: the spherical coordinate to convert,
                                given as a numpy array [latitude, longitude]
        :param reference_coord: the reference coordinate to use for conversion,
                                given as a numpy array [latitude, longitude]
        :returns: the approximated cartesian coordinates in meters, given as a numpy array [x, y, z]
        """
        # TODO


def main():
    # initialize the node
    rospy.init_node("localization")

    # create and start our localization system
    localization = Localization()

    # let the callback functions run asynchronously in the background
    rospy.spin()


if __name__ == "__main__":
    main()
