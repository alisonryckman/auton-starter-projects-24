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
        tag_loc = self.context.env.get_fid_data()

        # TODO: if we don't have a tag: go to the DoneState (with outcome "failure")
        if tag_loc is None:
            return "failure"

        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")
        in_dist_tol = (1 - tag_loc.closenessMetric) > DISTANCE_TOLERANCE
        in_ang_tol = (tag_loc.x / 10.0) < ANUGLAR_TOLERANCE

        if in_dist_tol and in_ang_tol:
            return "success"

        # TODO: figure out the Twist command to be applied to move the rover closer to the tag
        twist_command = Twist()
        if not in_dist_tol:
            twist_command.linear.x = tag_loc.closenessMetric
        if not in_ang_tol:
            twist_command.angular.z = -1 * tag_loc.x

        # TODO: send Twist command to rover
        self.context.rover.send_drive_command(twist_command)

        # TODO: stay in the TagSeekState (with outcome "working")

        return "working"
