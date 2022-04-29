import rospy
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import  MultiDOFJointTrajectory 

def CallbackPoint(data):
    tracker_input_pose_publisher = rospy.Publisher("/red/tracker/input_pose", PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    
    tracker_input_pose_publisher.publish(data)
    rate.sleep()

def CallbackTrajectory(data):
    tracker_input_pose_publisher = rospy.Publisher("/red/tracker/input_trajectory", MultiDOFJointTrajectory , queue_size=10)

    rate = rospy.Rate(10)
    
    tracker_input_pose_publisher.publish(msg)
    rate.sleep()

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("PosePublish", PoseStamped, CallbackPoint)
    rospy.Subscriber("TrajectoryPublish", MultiDOFJointTrajectory, CallbackTrajectory)

    rospy.spin()
