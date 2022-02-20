
import rospy

from std_msgs.msg import *
from mavros_msgs.msg import *

class MavrosState:
    def __init__(self):
        self.sub = rospy.Subscriber("/uav1/mavros/state", State, self.callback_mavros_states)
        self.armed = bool()
        self.mode = String()


    def callback_mavros_states(self,msg):
        self.armed = msg.armed
        self.mode = msg.mode 