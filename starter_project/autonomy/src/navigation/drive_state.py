import numpy as np

from context import Context
from drive import get_drive_command
from state import BaseState


class DriveState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["driving_to_point", "reached_point"],
        )

    def evaluate(self, ud):
        target = np.array([5.5, 2.0, 0.0])

        roverPose = self.context.rover.get_pose  # get the rover pose
        if roverPose == None:  # if the pose doesn't exist, stay in the drive state
            return "driving_to_point"

        # compute new drive command and whether we're at our target
        driveState = get_drive_command(target, roverPose, 0.7, 0.2)

        if driveState[1] == True:  # go to tag seek state, we've reached our target
            return "reached_point"
        else:  # we're not at our target yet, send new drive command to rover and stay in driving state
            self.context.rover.send_drive_command(driveState[0])
            return "driving_to_point"

        pass
