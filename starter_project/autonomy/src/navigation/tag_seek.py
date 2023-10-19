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
        DISTANCE_TOLERANCE = 0.4
        ANUGLAR_TOLERANCE = 0.05
        # TODO: get the tag's location and properties (HINT: use get_fid_data() from context.env)
        tag = self.context.env.get_fid_data()
        # TODO: if we don't have a tag: go to the DoneState (with outcome "failure")
        if tag.tagId == -1:
            return "failure"
        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")
        angle = tag.xTagCenterPixel * 3.14
        print("closeness:", tag.closenessMetric, "angle:", angle)
        if tag.closenessMetric < DISTANCE_TOLERANCE and abs(angle) < ANUGLAR_TOLERANCE and tag.closenessMetric != 0: # this isn't really an angle
            return "success"
        # TODO: figure out the Twist command to be applied to move the rover closer to the tag
        twist = Twist()
        # x is on [-0.5, 0.5] which corresponds to angles from [-pi\2, pi\2]
        # SUPER SCUFFED
        if abs(angle) > ANUGLAR_TOLERANCE: # we haven't rotated enough yet
            # twist.angular.z = 0
            twist.angular.z = -3 * angle # p controller, check if sign is right
        else:
            twist.linear.x = 0.2 # this is pretty arbitrary
        # TODO: send Twist command to rover
        # print(twist)
        self.context.rover.send_drive_command(twist)
        # TODO: stay in the TagSeekState (with outcome "working")
        return "working"
