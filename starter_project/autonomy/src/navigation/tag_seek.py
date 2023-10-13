from geometry_msgs.msg import Twist

from context import Context
from state import BaseState
import numpy as np


class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            # TODO: add outcomes
            add_outcomes=["failure","success","working"],
        )

    def evaluate(self, ud):
        DISTANCE_TOLERANCE = 0.99
        ANUGLAR_TOLERANCE = 0.3
        # TODO: get the tag's location and properties (HINT: use get_fid_data() from context.env)
        tag_location = self.context.env.get_fid_data()

        # TODO: if we don't have a tag: go to the DoneState (with outcome "failure")
        if (not tag_location):
            return "failure"

        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")
        rover_pose = self.context.rover.get_pose()
        
        distance_to_target = tag_location.closenessMetric
        #angle_to_target = np.arccos((tag_location * rover_pose.position)/(abs(tag_location)*abs(rover_pose.position)))
        angle_to_target = tag_location.xTagCenterPixel#gets number between -1 and 1

        if ((distance_to_target < DISTANCE_TOLERANCE) 
            & (abs(angle_to_target) < ANUGLAR_TOLERANCE)):
            return "success"
        
        # TODO: figure out the Twist command to be applied to move the rover closer to the tag

        twist  = Twist()


        twist.angular.z = angle_to_target

        twist.linear.x = 1

        # TODO: send Twist command to rover
        self.context.rover.send_drive_command(twist)
        # TODO: stay in the TagSeekState (with outcome "working")

        return "working"