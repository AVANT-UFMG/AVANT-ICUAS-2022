# DroneCMD.py
# Esse arquivo apresenta a classe DroneCMD que contem todas as funçõs de manipulação do movimento do drone

import rospy
from typing import List

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from custom_msgs_srvs.msg import TrajectoryHandle, Vector3
from custom_msgs_srvs.srv import TrajectoryHandleService, TrajectoryHandleServiceRequest, TrajectoryHandleServiceResponse, CheckPosition, CheckPositionRequest, CheckPositionResponse, GoTo, GoToRequest, GoToResponse

class DroneCMD():

    def __init__(self, delta: float=0.5) -> None:
        """
        Initializa ao services, services proxis, publisher, subscribers and other utils attributes
        @delta: float
            Precision to verify if drone position is correct
        """
        if delta < 0:
            raise "delta param must be positive"

        rospy.Service('/red/avant_cmd/go_to', GoTo, self.go_to)
        rospy.Service('/red/avant_cmd/exec_trajectory', TrajectoryHandleService, self.exec_trajectory)
        rospy.Service('/red/avant_cmd/check_position', CheckPosition, self.check_position)

        self.tracker_input_pose_publisher = rospy.Publisher("/red/tracker/input_pose", PoseStamped, queue_size=10)

        rospy.Subscriber("/red/odometry", Odometry, self.odometry_callback, queue_size=10)

        self.esimated_drone_pos: Vector3 = Vector3(-10, 0, 2.54)
        self.delta = delta


    def go_to(self, point: GoToRequest) -> GoToResponse:
        """
        A service to go to a simple point in the arena
        @point: GoToRequest -> point: Vector3
            Contain a points to be visited, return success if the drone visited successfully
        """
        position = PoseStamped()
        position.pose.position = point.point

        rate = rospy.Rate(10)
        i = 0
        while True:
            self.tracker_input_pose_publisher.publish(position)
            rate.sleep()
            i = i + 1
            if i > 1:
                break

        while not self.__check_position_func(Vector3(point.point.x, point.point.y, point.point.z)):
            rate.sleep()
        
        return GoToResponse(True, "Delivered on the point!")

    def exec_trajectory(self, trajectory: TrajectoryHandleServiceRequest) -> TrajectoryHandleServiceResponse:
        """
        A service to execute a trajectory
        @trajectory: TrajectoryHandleServiceRequest -> points: Vector3[]
            Contain a list of points to be visited, return success if the path complete successfully
        """
        points: List[Vector3] = trajectory.points
        position = PoseStamped()

        rate = rospy.Rate(10)

        for target in points:
            position.pose.position = target

            i = 0
            while True:
                self.tracker_input_pose_publisher.publish(position)
                rate.sleep()
                i = i + 1
                if i > 1:
                    break

            while not self.__check_position_func(target):
                rate.sleep()

        return TrajectoryHandleServiceResponse(True, "Trajectory complete!")

    def check_position(self, point: CheckPositionRequest) -> CheckPositionResponse:
        """
        A simple service to verify the drone is near the point requested
        @point: CheckPositionRequest -> position: Vector3
            Contain a position that need to be verified based on odometry
        """
        if abs(point.position.x - self.esimated_drone_pos.x) > self.delta or abs(point.position.y - self.esimated_drone_pos.y) > self.delta or abs(point.position.z - self.esimated_drone_pos.z) > self.delta:
            return CheckPositionResponse(False, "Drone out of target!")

        return CheckPositionResponse(True, "Drone on position!")

    def __check_position_func(self, point: Vector3) -> bool:
        """
        A private function to verify the drone is near the point requested
        @point: Vector3
            Is the position that need to be verified based on odometry
        """
        if abs(point.x - self.esimated_drone_pos.x) > self.delta or abs(point.y - self.esimated_drone_pos.y) > self.delta or abs(point.z - self.esimated_drone_pos.z) > self.delta:
            return False

        return True

    def odometry_callback(self, odom: Odometry) -> None:
        """
        A subscriber to save the x, y, z coordinates on the object to the others functions
        @odom: Odometry
            A complex tipe do odometry, that calback simplifies their information
        """
        self.esimated_drone_pos = Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)

if __name__ == "__main__":
    rospy.init_node("avant_cmd")
    DroneCMD()
    rospy.spin()