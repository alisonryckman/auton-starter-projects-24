from geometry_msgs.msg import Twist

from context import Context
from state import BaseState
import numpy as np


class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["working", "success", "failure"],
        )

    def evaluate(self, ud):
        DISTANCE_TOLERANCE = 0.99
        ANUGLAR_TOLERANCE = 50  # changed this to a pixel measurement; ideally we'd have information about the
        # image size so we could compute a band of acceptable pixel values using a percentage of the overall image size,
        # but that isn't the case here so this is just kind of a guess

        # get the tag's location and properties
        tagData = self.context.env.get_fid_data()

        # if we don't have a tag: go to the DoneState (with outcome "failure")
        if tagData == None:
            return "failure"

        # as far as an angular reference between the tag and rover goes, we only care about the x-axis of the tag; in other words, the
        # angular tolerance is concerned with the azimuth (related to x tag position), but not the altitutde (related to y tag position)

        if tagData.closenessMetric >= DISTANCE_TOLERANCE:
            withinDistance = True
        else:
            withinDistance = False
        if abs(tagData.xTagCenterPixel) > ANUGLAR_TOLERANCE:
            withinAngular = False
        else:
            withinAngular = True

        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")
        if withinDistance & withinAngular:
            return "success"
        else:
            # these are two parameters that adjust the "intensity" of the drive commands, since our error metrics aren't directly related to drive behavior
            angularScaleFactor = 1
            linearScaleFactor = 1

            driveSignal = Twist()  # create a new twist command
            if (
                withinAngular
            ):  # we don't want to change anything if we're within our angular tolerance, so check that first
                # angular command is proportional (in opposite direction) to angular error
                driveSignal.angular.z = 0
            else:
                driveSignal.angular.z = angularScaleFactor * (-tagData.xTagCenterPixel)
            if withinDistance:  # only apply linear command if we're not within our distance tolerance
                driveSignal.linear.x = 0
            else:
                driveSignal.linear.x = linearScaleFactor * (
                    1 / tagData.closenessMetric
                )  # angular command is inversely proportional to closeness metric val (small closeness metric values give large commands)

            Context.rover.send_drive_command(driveSignal)

            return "working"
