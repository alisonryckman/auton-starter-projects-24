from geometry_msgs.msg import Twist

from context import Context
from context import np
from state import BaseState
import math as math

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
        tagInfo = self.context.env.get_fid_data()
        twist = Twist()
        
        # TODO: if we don't have a tag: go to the DoneState (with outcome "failure")
        if (tagInfo == None):
            #self.addOutcomes.append("failure")
            return "failure"
        
        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")
        distance = tagInfo.closenessMetric
        angDist = tagInfo.xTagCenterPixel
        if (distance<DISTANCE_TOLERANCE and abs(angDist)<ANUGLAR_TOLERANCE):
            #self.addOutcomes.append("success")
            return "success"

        # TODO: figure out the Twist command to be applied to move the rover closer to the tag
        if (distance>=DISTANCE_TOLERANCE):
            twist.linear.x = 1
        if (angDist>=ANUGLAR_TOLERANCE):
            twist.angular.z = angDist
        

        # TODO: send Twist command to rover
        self.context.rover.send_drive_command(twist)

        # TODO: stay in the TagSeekState (with outcome "working")
        #self.addOutcomes.append("working")
        return "working"
