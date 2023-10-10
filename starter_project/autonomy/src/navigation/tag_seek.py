from geometry_msgs.msg import Twist

from context import Context
from state import BaseState


class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["working", "success", "failure"],
        )

    def evaluate(self, ud):
        DISTANCE_TOLERANCE = 0.99
        ANUGLAR_TOLERANCE = 0.3
        # get the tag's location and properties (HINT: use get_fid_data() from context.env)
        tagData = self.context.env.get_fid_data()

        # if we don't have a tag: go to the DoneState (with outcome "failure")
        if tagData == None:
            return "failure"

        # as far as an angular reference between the tag and rover goes, we only care about the x-axis of the tag; in other words, the
        # angular tolerance is concerned with the azimuth (related to x tag position), but not the altitutde (related to y tag position)

        withinAngular = False
        if tagData.closenessMetric >= DISTANCE_TOLERANCE:
            withinDistance = True
        else:
            withinDistance = False
        # if we create a triangle (from a top-down view) with the x-position of the tag and the rover lying at the ends of the hypotenuse,
        # then the angle we care about is between the horizontal leg and the hypotenuse of the triangle

        # I think i'm overcomplicating this a bit

        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")

        # TODO: figure out the Twist command to be applied to move the rover closer to the tag

        # TODO: send Twist command to rover

        # TODO: stay in the TagSeekState (with outcome "working")

        pass
