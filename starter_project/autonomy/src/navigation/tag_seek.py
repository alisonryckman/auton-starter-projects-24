from geometry_msgs.msg import Twist

from context import Context
from state import BaseState


class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            # TODO: add outcomes
            add_outcomes=["failure", "success", "working"],
        )

    def evaluate(self, ud):
        DISTANCE_TOLERANCE = 0.99
        ANUGLAR_TOLERANCE = 0.3
        # TODO: get the tag's location and properties (HINT: use get_fid_data() from context.env)
        dist = self.context.env.get_fid_data().closenessMetric
        x_coor = self.context.env.get_fid_data().xTagCenterPixel
        y_coor = self.context.env.get_fid_data().yTagCenterPixel
        
        # TODO: if we don't have a tag: go to the DoneState (with outcome "failure")
        if self.context.env.get_fid_data() is None:
            return "failure"
        if dist < DISTANCE_TOLERANCE and x_coor < ANUGLAR_TOLERANCE:
            return "success"
        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")
        temp = Twist()
        # change the linear.x value and/or the angular.z value
        if dist >= DISTANCE_TOLERANCE:
            temp.linear.x = 1
        if x_coor >= ANUGLAR_TOLERANCE:
            temp.angular.z = x_coor
        # TODO: figure out the Twist command to be applied to move the rover closer to the tag

        # TODO: send Twist command to rover
        self.context.rover.send_drive_command(temp)
        # TODO: stay in the TagSeekState (with outcome "working")
        return "working"