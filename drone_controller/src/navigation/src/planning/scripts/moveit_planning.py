#!/usr/bin/env python
#coding: utf-8

import sys

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import MultiDOFJointTrajectory


class Planner:
    def __init__(self, move_group_name):
        self.robot_pose = None

        self.move_group = moveit_commander.MoveGroupCommander(move_group_name)
        self.robot = moveit_commander.RobotCommander()

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.goal_sub = rospy.Subscriber('goal', PoseStamped, self.goal_callback)
        self.trajectory_pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=10)

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
    
    def set_start_state(self, pose):
        pos = pose.position
        ori = pose.orientation
        joint_state = JointState()
        joint_state.name = ['virtual_joint/trans_x', 'virtual_joint/trans_y', 'virtual_joint/trans_z', 'virtual_joint/rot_x', 'virtual_joint/rot_y', 'virtual_joint/rot_z', 'virtual_joint/rot_w']
        joint_state.position = [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        self.move_group.set_start_state(robot_state)

    def set_goal_state(self, goal):
        pos = goal.pose.position
        ori = goal.pose.orientation
        goal_state = [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
        self.move_group.set_joint_value_target(goal_state)
    
    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
 
    def goal_callback(self, msg):
        self.plan(msg)
    
    def plan(self, goal_pose):
        if self.robot_pose is None:
            rospy.logerr('Robot pose is unknown. Unable to plan')
            return
        
        self.set_start_state(self.robot_pose)
        self.set_goal_state(goal_pose)
        result = self.move_group.plan()
        if result[0] == True:
            self.trajectory_pub.publish(result[1].multi_dof_joint_trajectory)


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_planning', anonymous=True)

# move_group_name = rospy.get_param('move_group_name', default="drone")

planner = Planner('drone')
planner.set_planning_pipeline()
planner.set_planner()
planner.set_planning_time()
planner.set_num_attempts()
planner.set_workspace()

rospy.spin()
