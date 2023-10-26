import numpy as np

from context import Context
from drive import get_drive_command
from state import BaseState
import rospy as rp


class DriveState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["driving_to_point", "reached_point"],
        )

    def evaluate(self, ud):
        target = np.array([5.5, 2.0, 0.0])

        roverPose = self.context.rover.get_pose()  # get the rover pose
        tagData = self.context.env.get_fid_data()

        if roverPose == None:  # if the pose doesn't exist, stay in the drive state
            rp.loginfo("Pose doesn't exist")
            return "driving_to_point"
        else:
            rp.loginfo(roverPose.position[0])
            # compute new drive command and whether we're at our target
            driveCmd, status = get_drive_command(target, roverPose, 0.7, 0.2)

            if status:  # go to tag seek state, we've reached our target
                if tagData is not None:
                    if tagData.tagId != -1:  # log tag data at the state transition point
                        rp.loginfo(tagData.xTagCenterPixel)
                        rp.loginfo(tagData.yTagCenterPixel)
                return "reached_point"
            else:  # we're not at our target yet, send new drive command to rover and stay in driving state
                self.context.rover.send_drive_command(driveCmd)
                return "driving_to_point"
