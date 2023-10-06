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
        tag = self.context.env.get_fid_data()
        # TODO: if we don't have a tag: go to the DoneState (with outcome "failure")
        if tag is None:
            return "failure"
        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")
        if tag.closenessMetric > DISTANCE_TOLERANCE and abs(tag.xTagCenterPixel) < ANUGLAR_TOLERANCE:
            return "success"
        # TODO: figure out the Twist command to be applied to move the rover closer to the tag
        command = Twist()
        command.angular.z = tag.xTagCenterPixel
        command.linear.x = 1 - tag.closenessMetric
        # TODO: send Twist command to rover
        self.context.rover.send_drive_command(command)
        # TODO: stay in the TagSeekState (with outcome "working")
        return "working"
