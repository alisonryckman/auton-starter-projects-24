from geometry_msgs.msg import Twist

from context import Context
from state import BaseState


class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["success", "failure", "working"],
        )

    def evaluate(self, ud):
        DISTANCE_TOLERANCE = 0.99
        ANUGLAR_TOLERANCE = 0.3

        tag = self.context.env.get_fid_data()
        if tag == None:
            return "failure"

        if tag.closenessMetric < DISTANCE_TOLERANCE and (
            pow(tag.xTagCenterPixel, 2) + pow(tag.yTagCenterPixel, 2)
        ) < pow(ANUGLAR_TOLERANCE, 2):
            return "success"

        # TODO: figure out the Twist command to be applied to move the rover closer to the tag
        linear_cmp = [0, 0, 0]
        angular_cmp = [0, 0, 0]

        if tag.closenessMetric < DISTANCE_TOLERANCE:
            linear_cmp = [1, 0, 0]
        if pow(tag.xTagCenterPixel, 2) + pow(tag.yTagCenterPixel, 2) < pow(ANUGLAR_TOLERANCE, 2):
            if tag.xTagCenterPixel > 0:
                angular_cmp = [0, 0, 1]
            else:
                angular_cmp = [0, 0, -1]

        self.context.rover.send_drive_command(Twist(linear_cmp, angular_cmp))
        return "working"
