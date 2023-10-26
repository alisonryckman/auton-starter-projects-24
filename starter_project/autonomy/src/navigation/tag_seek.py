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
        DISTANCE_TOLERANCE = 1.0
        ANGULAR_TOLERANCE = 0.5
        # TODO: get the tag's location and properties (HINT: use get_fid_data() from context.env)

        tagLoc = self.context.env.get_fid_data()

        # TODO: if we don't have a tag: go to the DoneState (with outcome "failure")

        if tagLoc == None:
            return "failure"

        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")

        if tagLoc.closenessMetric < DISTANCE_TOLERANCE and tagLoc.xTagCenterPixel < ANGULAR_TOLERANCE:
            return "success"

        # TODO: figure out the Twist command to be applied to move the rover closer to the tag
        twist = Twist()

        if tagLoc.closenessMetric >= DISTANCE_TOLERANCE:
            twist.linear.x = 1

        if abs(tagLoc.xTagCenterPixel) >= ANGULAR_TOLERANCE:
            twist.angular.z = tagLoc.xTagCenterPixel

        # TODO: send Twist command to rover

        self.context.rover.send_drive_command(twist)

        # TODO: stay in the TagSeekState (with outcome "working")

        return "working"
