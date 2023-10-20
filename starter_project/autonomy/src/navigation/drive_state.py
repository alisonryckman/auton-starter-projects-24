import numpy as np

from context import Context
from drive import get_drive_command
from state import BaseState


class DriveState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            # TODO:
            add_outcomes=["driving_to_point", "reached_point"],
        )

    # function get_drive_command() imported from drive.py that
    # you can use to do some of the math in this step for you.

    # The function should return a string "outcome" depending on what state it needs to transition to next.
    # need to add those outcomes to the add_outcomes parameter that is passed to the parent constructor.

    def evaluate(self, ud):
        target = np.array([5.5, 2.0, 0.0])
        # TODO: get the rover's pose, if it doesn't exist stay in DriveState (with outcome "driving_to_point")
        roverPose = self.context.rover.get_pose()
        if roverPose is None:
            return "driving_to_point"
        # TODO: get the drive command and completion status based on target and pose
        # (HINT: use get_drive_command(), with completion_thresh set to 0.7 and turn_in_place_thresh set to 0.2)
        drive_effort = get_drive_command(target, roverPose, 0.7, 0.2)
        # TODO: if we are finished getting to the target, go to TagSeekState (with outcome "reached_point")
        if drive_effort[1] == True:
            return "reached_point"
            # TODO: send the drive command to the rover
        self.context.rover.send_drive_command(drive_effort[0])
        # TODO: tell smach to stay in the DriveState by returning with outcome "driving_to_point"
        return "driving_to_point"
