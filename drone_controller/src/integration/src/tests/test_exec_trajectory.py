import rospy
from geometry_msgs.msg import PoseStamped
from custom_msgs_srvs.srv import TrajectoryHandleService, TrajectoryHandleServiceRequest
from custom_msgs_srvs.msg import Vector3

if __name__ == "__main__":
    rospy.init_node("test_exec_trajectory")
    test_exec_trajectory = rospy.ServiceProxy('/red/avant_cmd/exec_trajectory', TrajectoryHandleService)
    test_msg = TrajectoryHandleServiceRequest()
    test_msg.points = [Vector3(-9.0,0.0,2.5), Vector3(-3.0,0.0,2.5), Vector3(3.0,0.0,2.5), Vector3(9.0,0.0,2.5)]

    msg = test_exec_trajectory(test_msg)
    print(msg.message)

