#!/usr/bin/env python

import rospy
from custom_msgs_srvs.msg import TrajectoryHandle, Vector3

if __name__ == "__main__":
    rospy.init_node("test_custom_msgs")
    pub = rospy.Publisher("/test/list_of_points", TrajectoryHandle, queue_size=10)

    rate = rospy.Rate(5)
    
    while not rospy.is_shutdown():
        msg = TrajectoryHandle()
        msg.points = [Vector3(1, 2, 3), Vector3(1, 2, 3), Vector3(1, 2, 3), Vector3(1, 2, 3)]

        pub.publish(msg)

        rate.sleep()