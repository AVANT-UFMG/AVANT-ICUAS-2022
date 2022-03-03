import rospy
from geometry_msgs.msg import PointStamped

if __name__ == "__main__":
    rospy.init_node("test_input_pose_topic_by_code")
    tracker_input_pose_publisher = rospy.Publisher("/red/tracker/input_pose", PointStamped, queue_size=10)
    test_msg = PointStamped()
    test_msg.point.x = 9.0
    test_msg.point.y = 0.0
    test_msg.point.z = 2.5
    tracker_input_pose_publisher.publish(test_msg)
    rospy.spin()