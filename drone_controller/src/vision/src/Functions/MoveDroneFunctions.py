import rospy
import time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from Class import Point


def GetDronePoint():
    pointDrone =  rospy.wait_for_message('/red/odometry', Odometry) 
    return pointDrone.pose.pose

def ConvertGpsPoint(gpsPoint):
    msg = PoseStamped()
    msg.pose = gpsPoint

    return msg

def PublishDeadPoint():
    msg = GetDronePoint()

    rate = rospy.Rate(10) # 10hz
    pub = rospy.Publisher('PosePublish', PoseStamped, queue_size=100)

    pub.publish(ConvertGpsPoint(msg))
    time.sleep(2)
    rate.sleep()

def MoveToPoint(point):

    rate = rospy.Rate(10) # 10hz

    pub = rospy.Publisher('PosePublish', PoseStamped, queue_size=100)

    pub.publish(point.GetPoseStamped())
    time.sleep(10)
    rate.sleep()

def ComparePoints(point, gps_point):
    
    x_finish = (float(point.x - 0.5) < gps_point.position.x) and (float(point.x +0.5) >= gps_point.position.x)
    y_finish = (float(point.y - 0.5) < gps_point.position.y ) and (float(point.y +0.5) >= gps_point.position.y)
    z_finish = (float(point.z - 0.5) < gps_point.position.z ) and (float(point.z +0.5) >= gps_point.position.z)
    yaw_finish = (float(point.yaw - 0.4) < gps_point.orientation.z ) and (float(point.yaw +0.4) >= gps_point.orientation.z)

   
    return not (x_finish and y_finish and z_finish and yaw_finish)

def ArrivePoint(point):
    arrive = True
    i = 0

    while(arrive):
        i = i +1

        gps_info = GetDronePoint()
        arrive = ComparePoints(point, gps_info)

        if(i>300):  
            print("time limit")
            break
