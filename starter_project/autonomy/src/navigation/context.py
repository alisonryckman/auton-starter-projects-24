from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from mrover.msg import StarterProjectTag
from util.SE3 import SE3
from visualization_msgs.msg import Marker


@dataclass
class Rover:
    ctx: Context

    def get_pose(self) -> Optional[SE3]:
        return SE3.from_tf_tree(self.ctx.tf_buffer, "map", "base_link")

    def send_drive_command(self, twist: Twist):
        self.ctx.vel_cmd_publisher.publish(Twist)  # publish the twist using the publisher defined in Context

    def send_drive_stop(self):
        stopTwist = Twist()  # initializes to zero; could also do this directly in the send drive statement
        self.send_drive_command(stopTwist)  # send zero twist using send_drive_command


@dataclass
class Environment:
    """
    Context class to represent the rover's environment
    Information such as locations of fiducials or obstacles
    """

    ctx: Context
    fid_pos: Optional[StarterProjectTag]

    def receive_fid_data(self, message: StarterProjectTag):
        self.fid_pos = StarterProjectTag

    def get_fid_data(self) -> Optional[StarterProjectTag]:
        """
        Retrieves the last received message regarding fid pose
        if it exists, otherwise returns None
        """
        if self.fid_pos.tagId == -1:  # recall that in perception.cpp,
            # we defined an empty tag vector (no tags) as corresponding to a tag id of -1
            return None
        else:  # if the tag id isn't -1, it does exist and we should simply return it
            return self.fid_pos


class Context:
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    vel_cmd_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher
    fid_listener: rospy.Subscriber

    # Use these as the primary interfaces in states
    rover: Rover
    env: Environment

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_cmd_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher("nav_vis", Marker)
        self.rover = Rover(self)
        self.env = Environment(self, None)
        self.fid_listener = rospy.Subscriber("/tag", StarterProjectTag, self.env.receive_fid_data)
