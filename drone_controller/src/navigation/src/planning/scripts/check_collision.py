#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import rospy
import math
from std_msgs.msg import Bool, Float32

def distance_to_color(x):
    b = -0.19252389931978442
    a = 0.9983750292387534
    y = a*x+b
    return y

def color_to_distance(y):
    b = -0.19252389931978442
    a = 0.9983750292387534
    x = (y-b)/a
    return x

bridge = CvBridge() 

depth_map = []
def camera_depth_callback(data):
    global depth_map
    cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
    depth_map = cv_image

node_name = 'colision_check'
rospy.init_node(node_name)
print(f'Node {node_name} iniciado com sucesso')

rospy.Subscriber('/red/camera/depth/image_raw', Image, callback=camera_depth_callback)

collision_pub = rospy.Publisher("collision", Bool, queue_size=10)
distance_pub = rospy.Publisher("distance_to_closest_obstacle", Float32, queue_size=10)

while(not rospy.is_shutdown()):

    if len(depth_map) == 0: continue

    y1 = 0
    y2 = int(depth_map.shape[0]-1)
    x1 = int(depth_map.shape[1] / 8 + 22)
    x2 = int(7 * depth_map.shape[1] / 8 - 22)

    reduced_depth_map = depth_map[ y1 : y2 , x1 : x2 ]

    cv2.imshow('rdm',reduced_depth_map)
    cv2.waitKey(1)

    collision_msg = Bool(data=False)
    # collision_msg.data = False

    max_dist = 5
    dist_msg = Float32(data=max_dist)

    for i in range(reduced_depth_map.shape[0]-1):
        for j in range(reduced_depth_map.shape[1]-1):
            if not math.isnan(reduced_depth_map[i][j]) and reduced_depth_map[i][j] < distance_to_color(max_dist):
                collision_msg.data = True
                dist_msg.data = color_to_distance(reduced_depth_map[i][j])
                break
        if collision_msg.data == True:
            break
    
    collision_pub.publish(collision_msg)
    distance_pub.publish(dist_msg)

