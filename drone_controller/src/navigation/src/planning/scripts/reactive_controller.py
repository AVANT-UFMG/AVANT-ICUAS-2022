#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Point

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np

class ReactiveController:
    def __init__(self, max_step=5.0, min_dist=2.0):
        self.pos = None
        self.ori = None
        self.euler = [0, 0, 0]

        self.max_step = max_step     # passo a cada iteracao em metros
        self.min_dist = min_dist
        self.collision_detected = False     # False pq na zona 1 nao tem obstaculos
        self.closest_obs_dist = -1
        
        self.pose_pub = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)

        odom_sub = rospy.Subscriber('/red/odometry', Odometry, callback=self.odom_cb)
        collision_sub = rospy.Subscriber('collision', Bool, callback=self.collision_cb)
        distance_sub = rospy.Subscriber('distance_to_closest_obstacle', Float32, callback=self.dist_cb)

    def odom_cb(self, msg: Odometry):
        self.pos = msg.pose.pose.position
        self.ori = msg.pose.pose.orientation
        self.euler = euler_from_quaternion([self.ori.x, self.ori.y, self.ori.z, self.ori.w])

    def collision_cb(self, msg: Bool):
        self.collision_detected = msg.data

    def dist_cb(self, msg: Float32):
        self.closest_obs_dist = msg.data

    def turn(self, angle):
        quat = quaternion_from_euler(0, 0, angle)
        ori = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        goal = PoseStamped()
        goal.pose.position = Point(x=self.pos.x, y=self.pos.y, z=self.pos.z)
        goal.pose.orientation = ori

        controller.pose_pub.publish(goal)

        goal = angle
        while True:
            yaw = self.euler[2]
            if np.abs(yaw - angle) < np.deg2rad(10):
                break
        rospy.sleep(1)

    def go(self, step, siri=False, offset=1.5):
        goal = PoseStamped()
        goal.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        if not siri:
            goal.pose.position.x = self.pos.x + step - offset
            goal.pose.position.y = self.pos.y
            goal.pose.position.z = self.pos.z
        else:
            goal.pose.position.x = self.pos.x
            goal.pose.position.y = self.pos.y + step - offset
            goal.pose.position.z = self.pos.z
            
        controller.pose_pub.publish(goal)

        goal_p = np.array([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])
        while True:
            pos = np.array([self.pos.x, self.pos.y, self.pos.z])
            if np.linalg.norm(goal_p - pos) < 0.5:
                break
            rospy.sleep(1/10)

    def leave_obstacle(self):
        dist_check = 3
        direction = -np.sign(self.pos.y)
        self.turn(direction*np.deg2rad(90))
        dist_left = self.closest_obs_dist
        if dist_left >= dist_check:
            self.go(direction*2.0, siri=True, offset=0.5)
            self.turn(0)
            return

        self.turn(np.deg2rad(-90))
        dist_right = self.closest_obs_dist
        if dist_right >= dist_check:
            self.go(-direction*2.0, siri=True, offset=0.5)
            self.turn(0)
            return

        self.go(step=-1)

        self.turn(np.deg2rad(90))
        dist_left = self.closest_obs_dist
        if dist_left >= dist_check:
            self.go(direction*2.0, siri=True, offset=0.5)
            self.turn(0)
            return

        self.turn(np.deg2rad(-90))
        dist_right = self.closest_obs_dist
        if dist_right >= dist_check:
            self.go(-direction*2.0, siri=True, offset=0.5)
            self.turn(0)
            return
        
        
rospy.init_node('reactive_controller')

controller = ReactiveController()

start_launch_ball = rospy.Publisher('/Task3Controller/Start', Bool, queue_size=10)

def startProjectCallback(msg: Bool):
    if msg.data:

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            if controller.pos is None or controller.ori is None:
                rospy.loginfo('Waiting for odometry data.')
                continue

            if controller.closest_obs_dist == -1:
                rospy.loginfo('Waiting for sensor data.')
                continue

            next_pose = PoseStamped()
            if controller.closest_obs_dist < controller.min_dist:
                controller.leave_obstacle()
            elif controller.closest_obs_dist < controller.max_step:
                controller.go(controller.closest_obs_dist)
            else:
                controller.go(5.0)

            if controller.pos.x > 0:
                    break
            
            rate.sleep()

        start_launch_ball.publish(Bool(True))


rospy.Subscriber("/red/challenge_started", Bool, startProjectCallback)

rospy.spin()