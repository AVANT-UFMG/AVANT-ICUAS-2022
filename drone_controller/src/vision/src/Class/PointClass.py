from geometry_msgs.msg import PoseStamped, Transform ,Twist ,Vector3 , Quaternion 
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory

class Point():
    x = 0.0
    y = 0.0
    z = 0.0
    yaw = 0.0

    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw 

    def GetPoseStamped(self):
        msg = PoseStamped()
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        msg.pose.orientation.z = self.yaw
        msg.pose.orientation.w = 1.0

        return msg

    def GetTrajectoryPoint(self):
        transforms = Transform()
        transforms.translation.x = self.x
        transforms.translation.y = self.y
        transforms.translation.z = self.z
        transforms.rotation.x = 0.0
        transforms.rotation.y = 0.0
        transforms.rotation.z = self.yaw
        transforms.rotation.w = 1.0

        velocities = Twist()
        accelerations = Twist()

        # time_from_start_var = rospy.Duration(10.0)

        pointTrajectory = MultiDOFJointTrajectoryPoint()
        pointTrajectory.transforms = [transforms]
        pointTrajectory.velocities = [velocities]
        pointTrajectory.accelerations = [accelerations]

        return pointTrajectory