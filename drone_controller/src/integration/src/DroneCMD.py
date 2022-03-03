# DroneCMD.py
# Esse arquivo apresenta a classe DroneCMD que contem todas as funçõs de manipulação básica do movimento do drone

import rospy

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from custom_msgs_srvs.msg import TrajectoryHandle, Vector3
from custom_msgs_srvs.srv import TrajectoryHandleService, TrajectoryHandleServiceRequest, TrajectoryHandleServiceResponse, CheckPosition, CheckPositionRequest, CheckPositionResponse, GoTo, GoToRequest, GoToResponse

class DroneCMD():

    def __init__(self, delta=0.1) -> None:
        rospy.Service('/red/avant_cmd/go_to', GoTo, self.go_to)
        rospy.Service('/red/avant_cmd/exec_trajectory', TrajectoryHandleService, self.exec_trajectory)
        rospy.Service('/red/avant_cmd/check_position', CheckPosition, self.check_position)

        self.tracker_input_pose_publisher = rospy.Publisher("/red/tracker/input_pose", PointStamped, queue_size=10)

        rospy.Subscriber("/red/odometry", Odometry, self.odometry_callback, queue_size=10)

        self.esimated_drone_pos: Vector3 = Vector3(-10, 0, 2.54)
        self.delta = delta


    def go_to(self, point: GoToRequest) -> GoToResponse:
        position = PointStamped()
        position.point = point.point

        rate = rospy.Rate(5)
        while not self.__check_position_func(Vector3(point.point.x, point.point.y, point.point.z)):
            self.tracker_input_pose_publisher.publish(position)
            rate.sleep()

        return GoToResponse(True, "Point achievement complete!")

    def exec_trajectory(self, trajectory: TrajectoryHandleServiceRequest) -> TrajectoryHandleServiceResponse:
        pass

    def check_position(self, point: CheckPositionRequest) -> CheckPositionResponse:
        if abs(point.position.x - self.esimated_drone_pos.x) > self.delta or abs(point.position.y - self.esimated_drone_pos.y) > self.delta or abs(point.position.z - self.esimated_drone_pos.z) > self.delta:
            return CheckPositionResponse(False, "Drone out of target!")

        return CheckPositionResponse(True, "Drone on position!")

    def __check_position_func(self, point: Vector3) -> bool:
        if abs(point.x - self.esimated_drone_pos.x) > self.delta or abs(point.y - self.esimated_drone_pos.y) > self.delta or abs(point.z - self.esimated_drone_pos.z) > self.delta:
            return False

        return True

    def odometry_callback(self, odom: Odometry) -> None:
        self.esimated_drone_pos = Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)

if __name__ == "__main__":
    rospy.init_node("avant_cmd")
    DroneCMD()
    rospy.spin()