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
        tag_Loc = self.context.env.get_fid_data()
        # TODO: if we don't have a tag: go to the DoneState (with outcome "failure")
        if tag_Loc == None:
            return "failure"
        distance_to_target = tag_Loc.closenessMetric
        angle_to_target = tag_Loc.xTagCenterPixel
        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")
        if ((distance_to_target < DISTANCE_TOLERANCE)) and (abs(angle_to_target) < ANUGLAR_TOLERANCE):
            return "success"
        # TODO: figure out the Twist command to be applied to move the rover closer to the tag
        twist = Twist()

        # TODO: send Twist command to rover

        if abs(angle_to_target) >= ANUGLAR_TOLERANCE:
            twist.angular.z = angle_to_target
        if abs(distance_to_target) >= DISTANCE_TOLERANCE:
            twist.linear.x = 1

        # TODO: stay in the TagSeekState (with outcome "working")
        self.context.rover.send_drive_command(twist)
        return "working"
