#!/usr/bin/env python
#coding: utf-8

import sys

import rospy
import tf2_geometry_msgs
import moveit_commander
from geometry_msgs.msg import Point, Pose, PoseStamped, Transform, TransformStamped, Vector3, Quaternion
from moveit_msgs.msg import LinkPadding, LinkScale, PlanningScene, RobotState
from nav_msgs.msg import Odometry
from octomap_msgs.msg import Octomap, OctomapWithPose
from sensor_msgs.msg import JointState, MultiDOFJointState
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class Planner:
    def __init__(self, move_group_name):
        # data we need
        self.initial_pose = None
        self.initial_pose_transform = None
        self.robot_pose = None
        self.octomap = None

        # MoveIt!
        self.move_group = moveit_commander.MoveGroupCommander(move_group_name)
        self.robot = moveit_commander.RobotCommander()
        self.scene_interface = moveit_commander.PlanningSceneInterface()

        # publishers and subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.goal_sub = rospy.Subscriber('goal', PoseStamped, self.goal_callback)
        self.octomap_sub = rospy.Subscriber('octomap_full', Octomap, self.octomap_callback)
        self.trajectory_pub = rospy.Publisher('trajectory', MultiDOFJointTrajectory, queue_size=10)

    def __pose_to_joint_state(self, pose: PoseStamped):
        ''' Convert Pose message to JointState message. '''
        pos = pose.pose.position
        ori = pose.pose.orientation

        joint_state = JointState()
        joint_state.header = pose.header
        joint_state.name = ['virtual_joint/trans_x', 'virtual_joint/trans_y', 'virtual_joint/trans_z', 'virtual_joint/rot_x', 'virtual_joint/rot_y', 'virtual_joint/rot_z', 'virtual_joint/rot_w']
        joint_state.position = [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]

        return joint_state

    def __pose_to_multi_dof_joint_state(self, pose: PoseStamped):
        ''' Convert Pose message to MultiDOFJointState message. '''
        pos = pose.pose.position
        ori = pose.pose.orientation

        transform = Transform()
        transform.translation = Vector3(x=pos.x, y=pos.y, z=pos.z)
        transform.rotation = ori

        joint_state = MultiDOFJointState()
        joint_state.joint_names = ['virtual_joint']
        joint_state.transforms = [transform]

        return joint_state

    def __pose_to_robot_state(self, pose: PoseStamped):
        ''' Convert Pose message to RobotState message. '''
        robot_state = RobotState()
        robot_state.joint_state = self.__pose_to_joint_state(pose)

        return robot_state

    def set_initial_pose(self, pose: PoseStamped):
        self.initial_pose = pose
        pos = pose.pose.position
        ori = pose.pose.orientation
        self.initial_pose_transform = TransformStamped()
        self.initial_pose_transform.header = pose.header
        self.initial_pose_transform.transform.translation = Vector3(x=pos.x, y=pos.y, z=pos.z)
        self.initial_pose_transform.transform.rotation = ori

    def set_planning_pipeline(self, pipeline='ompl'):
        self.move_group.set_planning_pipeline_id(pipeline)

    def set_planner(self, planner='BiTRRT'):
        self.move_group.set_planner_id(planner)

    def set_num_attempts(self, num_attempts=20):
        self.move_group.set_num_planning_attempts(num_attempts)

    def set_planning_time(self, time=10):
        self.move_group.set_planning_time(time)

    def set_workspace(self, ws=[-15, -15, 0, 15, 15, 10]):
        self.move_group.set_workspace(ws)

    def set_start_state(self, pose: PoseStamped):
        ''' Set start state of the planning problem. '''
        relative_pose = self.__get_relative_pose(pose)      
        robot_state = self.__pose_to_robot_state(relative_pose)
        self.move_group.set_start_state(robot_state)

    def set_goal_state(self, goal: PoseStamped):
        ''' Set goal state of the planning problem. '''
        relative_pose = self.__get_relative_pose(goal)
        goal_state = self.__pose_to_joint_state(relative_pose)
        self.move_group.set_joint_value_target(goal_state)
    
    def __get_relative_pose(self, pose: PoseStamped):
        ''' Return position relative to initial position.'''
        result = tf2_geometry_msgs.do_transform_pose(pose, self.initial_pose_transform)
        return result

    # TODO?
    def __get_relative_trajectory(self, trajectory: MultiDOFJointTrajectory):
        pass
        
    def __get_translated_map(self):
        pos = Point()
        ori = Quaternion(w=1)
        origin = PoseStamped()
        origin.pose.position = pos
        origin.pose.orientation = ori

        map = OctomapWithPose()
        map.header.frame_id = 'world'
        map.header.stamp = rospy.Time.now()
        map.origin = self.__get_relative_pose(origin).pose
        map.octomap = self.octomap

        return map

    def odom_callback(self, msg: Odometry):
        if self.initial_pose is None:
            initial_pose = PoseStamped()
            initial_pose.header = msg.header
            initial_pose.pose = msg.pose.pose
            self.set_initial_pose(initial_pose)
        
        if self.robot_pose is None:
            self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
 
    def goal_callback(self, msg: PoseStamped):
        self.plan(msg)

    def octomap_callback(self, msg: Octomap):
        self.octomap = msg

    def update_planning_scene(self):
        if self.robot_pose is None:
            rospy.logerr('Robot position is unknown.')
            return

        scene = PlanningScene()
        scene.robot_state = self.__pose_to_robot_state(self.robot_pose)
        scene.link_padding = [LinkPadding(link_name=name, padding=0.0) for name in self.robot.get_link_names()]
        scene.link_scale = [LinkScale(link_name=name, scale=1.0) for name in self.robot.get_link_names()]
        scene.world.octomap = self.__get_translated_map()
        
        self.scene_interface.apply_planning_scene(scene)
    
    def plan(self, goal_pose):
        if self.robot_pose is None:
            rospy.logerr('Robot pose is unknown. Unable to plan')
            return
        
        self.update_planning_scene()
        self.set_start_state(self.robot_pose)
        self.set_goal_state(goal_pose)
        
        result = self.move_group.plan()
        if result[0] == True:
            # self.move_group.execute(result[1])
            trajectory = result[1].multi_dof_joint_trajectory
            trajectory.header.frame_id = 'world'
            trajectory.header.stamp = rospy.Time.now()
            self.trajectory_pub.publish(trajectory)


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_planning', anonymous=True)

planner = Planner('drone')
planner.set_planning_pipeline()
planner.set_planner()
planner.set_planning_time()
planner.set_num_attempts()
planner.set_workspace()

rospy.spin()
