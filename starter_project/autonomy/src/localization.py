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
        spherical_coord = np.array([msg.latitude, msg.longitude])
        reference_coord = np.array([42.293195, -83.7096706])
        converted_coords = self.spherical_to_cartesian(spherical_coord, reference_coord)
        self.pose = SE3(converted_coords, self.pose.rotation)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")


    def imu_callback(self, msg: Imu):
        """
        This function will be called every time this node receives an Imu message
        on the /imu topic. It should read the orientation data from the given Imu message,
        store that value in `self.pose`, then publish that pose to the TF tree.
        """
        rotation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.pose = SE3.from_pos_quat(self.pose.position, rotation)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")

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
        converted_coords = np.array([0.0, 0.0, 0.0])

        # Latitude
        distance = np.radians(spherical_coord[0]) - np.radians(reference_coord[0])
        conversion = distance * (6371000)
        converted_coords[0] = conversion
        # Longitude
        distance = np.radians(spherical_coord[1]) - np.radians(reference_coord[1])
        conversion = distance * (6371000)
        conversion = conversion * np.cos(np.radians(reference_coord[0]))
        converted_coords[1] = conversion
        # z-axis
        converted_coords[2] = 0

        # rospy.loginfo("Converted Coordinates: %d", converted_coords)
        temp = converted_coords[0]
        converted_coords[0] = converted_coords[1]
        converted_coords[1] = temp
        return converted_coords



def main():
    # initialize the node
    rospy.init_node("localization")

    # create and start our localization system
    localization = Localization()

    # let the callback functions run asynchronously in the background
    rospy.spin()


if __name__ == "__main__":
    main()
