#!/usr/bin/env python3

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy
import tf2_ros
import math

# ROS message types we need to use
from sensor_msgs.msg import NavSatFix, Imu

# SE3 library for handling poses and TF tree
from util.SE3 import SE3
from util.SO3 import SO3


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
        """
        This function will be called every time this node receives a NavSatFix message
        on the /gps topic. It should read the GPS location from the given NavSatFix message,
        convert it to cartesian coordinates, store that value in `self.pose`, then publish
        that pose to the TF tree.
        """
        #TODO
        #print(msg.latitude, msg.longitude)
        referenceLat = 42.293195
        referenceLong = -83.7096706

        spherical = np.array([msg.latitude, msg.longitude])
        ref = np.array([referenceLat, referenceLong])
        x = Localization.spherical_to_cartesian(spherical, ref)
        self.pose = SE3(position=x.copy(), rotation=self.pose.rotation)
        SE3.publish_to_tf_tree(self.pose, self.tf_broadcaster, "map", "base_link")

    def imu_callback(self, msg: Imu):
        """
        This function will be called every time this node receives an Imu message
        on the /imu topic. It should read the orientation data from the given Imu message,
        store that value in `self.pose`, then publish that pose to the TF tree.
        """
        y = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z,msg.orientation.w])
        self.pose = SE3.from_pos_quat(position=self.pose.position, quaternion=y)
        SE3.publish_to_tf_tree(self.pose, self.tf_broadcaster, "map", "base_link")
        

    @staticmethod
    def spherical_to_cartesian(spherical_coord: np.ndarray, reference_coord: np.ndarray) -> np.ndarray:
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
        #TODO
        referenceLat = np.radians(reference_coord[0])
        referenceLong = np.radians(reference_coord[1])
        crc = 6371000
        latDist = (
            np.radians(spherical_coord[0]) - referenceLat
        ) * np.radians(crc) # converts horizontal dist from degrees to meter, this becomes the x coordinate

        longDist = (
            (np.radians(spherical_coord[1]) - referenceLong) * crc * (np.cos(referenceLat))
        )  # converts longitude distance to meters
        return np.array([longDist, latDist, 0])


def main():
    # initialize the node
    rospy.init_node("localization")

    # create and start our localization system
    localization = Localization()

    # let the callback functions run asynchronously in the background
    rospy.spin()


if __name__ == "__main__":
    main()
