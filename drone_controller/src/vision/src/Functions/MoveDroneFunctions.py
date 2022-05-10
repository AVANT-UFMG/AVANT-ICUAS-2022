import rospy
import time
from geometry_msgs.msg import PoseStamped, Transform ,Twist ,Vector3 , Quaternion 
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory 
from nav_msgs.msg import Odometry
from Class import Point

DronePoint = None
def DronePointCallBack(data):
    global DronePoint
    DronePoint = data.pose.pose

rospy.Subscriber("/red/odometry", Odometry, DronePointCallBack)
pub = rospy.Publisher("/red/tracker/input_pose", PoseStamped, queue_size=10)

def GetDronePoint():
    global DronePoint
    r = rospy.Rate(100)
    while DronePoint is None: 
        r.sleep()

    return DronePoint

def ConvertGpsPoint(gpsPoint):
    msg = PoseStamped()
    msg.pose = gpsPoint

    return msg

def PublishDeadPoint():
    global pub

    msg = GetDronePoint()
    rate = rospy.Rate(10) # 10hz

    pub.publish(ConvertGpsPoint(msg))
    time.sleep(2)
    rate.sleep()

def MoveToPoint(point):
    global pub 
    timeW = 0
    
    rate = rospy.Rate(10)
    while pub.get_num_connections() < 1 :
        rate.sleep()
        if timeW > 500 : break
        print("wait")
        timeW += 1
    
    pub.publish(point.GetPoseStamped())
    time.sleep(6)
    ArrivePoint(point)

def ComparePoints(point, gps_point):
    x_finish = (float(point.x - 0.5) < gps_point.position.x) and (float(point.x +0.5) >= gps_point.position.x)
    y_finish = (float(point.y - 0.5) < gps_point.position.y ) and (float(point.y +0.5) >= gps_point.position.y)
    z_finish = (float(point.z - 0.5) < gps_point.position.z ) and (float(point.z +0.5) >= gps_point.position.z)
    
    return not (x_finish and y_finish and z_finish )

def WaitStablePosition():
    lastPoint = GetDronePoint()
    time.sleep(0.3)
    stable = False
    i = 0

    while(stable):
        i = i +1
        actualPoint = GetDronePoint()

        statusX = actualPoint.position.x - 0.05 < lastPoint.position.x and actualPoint.position.x + 0.05 >= lastPoint.position.y
        statusY = actualPoint.position.y - 0.05 < lastPoint.position.x and actualPoint.position.y + 0.05 >= lastPoint.position.y
        statusZ = actualPoint.position.z - 0.05 < lastPoint.position.x and actualPoint.position.z + 0.05 >= lastPoint.position.z
        statusYaw = actualPoint.orientation.z - 0.05 < lastPoint.orientation.z and actualPoint.orientation.z + 0.05 >= lastPoint.orientation.z

        stable = statusX and statusY and statusZ and statusYaw
        lastPoint = actualPoint
        time.sleep(0.3)

        if(i>100):  
            print("time limit")
            break

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
