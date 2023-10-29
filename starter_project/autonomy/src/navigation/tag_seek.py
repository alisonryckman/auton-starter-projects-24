from geometry_msgs.msg import Twist

from context import Context
from state import BaseState


class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["failure", "working", "success"],
        )

    def evaluate(self, ud):
        DISTANCE_TOLERANCE = 0.99
        ANUGLAR_TOLERANCE = 0.3

        tag = self.context.env.get_fid_data

        if tag is None:
            return "failure"

        if tag.closenessMetric < DISTANCE_TOLERANCE and abs(tag.xTagCenterPixel) < ANUGLAR_TOLERANCE:
            return "success"

        twist = Twist()
        if tag.closenessMetric >= DISTANCE_TOLERANCE:
            twist.linear.x = 1
        if abs(tag.xTagCenterPixel) >= ANUGLAR_TOLERANCE:
            twist.angular.z = tag.xTagCenterPixel
        
        self.context.rover.send_drive_command(twist)

        return "working"
