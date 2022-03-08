import rospy
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("test_input_pose_topic_by_code")
    tracker_input_pose_publisher = rospy.Publisher("/red/tracker/input_pose", PoseStamped, queue_size=10)
    test_msg = PoseStamped()
    test_msg.pose.position.x = 9.0
    test_msg.pose.position.y = 0.0
    test_msg.pose.position.z = 2.5
    rate = rospy.Rate(10)
    i = 0
    while True:
        tracker_input_pose_publisher.publish(test_msg)
        rate.sleep()
        i = i + 1
        if i > 1:
            break
