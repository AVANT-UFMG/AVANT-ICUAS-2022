import rospy

from std_srvs.srv import StringBoolean, BoolResponse

STATES = ['load_graph', 'apply_trajectory', 'see_walls', 'find_artag', 'publish_airtag' 'handle_ball']

class StateMachine():

    def __init__(self, initil_state='load_graph') -> None:
        """
        Initializa the services, services proxis, publisher, subscribers and other utils attributes
        """
        self.state = initil_state
        rospy.Service('/red/avant_cmd/set_state', StringBoolean, self.set_state)

    def set_state(self, msg) -> BoolResponse:
        """
        Change the state to change the drone goal
        """
        if msg.next_state in STATES:
            self.state = msg.next_state
            return True
        else:
            return False
        
