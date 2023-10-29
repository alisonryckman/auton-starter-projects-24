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

         pose = self.context.rover.get_pose()
        if pose is None:
            return "driving_to_point"
    
        drive_command, completion_status = get_drive_command(target, pose, 0.7, 0.2)

        if completion_status:
            return "reached_point"

        self.context.rover.get_drive_command(drive_command)
        return "driving_to_point"
