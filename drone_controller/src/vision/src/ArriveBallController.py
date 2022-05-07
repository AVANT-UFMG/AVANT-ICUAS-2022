import rospy

from trajectory_msgs.msg import  MultiDOFJointTrajectory 
from std_msgs.msg import Bool
from std_msgs.msg import Float32 
from geometry_msgs.msg import Pose

from Class import Point , Trajectory
from Functions import ( MoveToPoint, PublishDeadPoint )

PointToArriveTest = Point(7.5, 7.5, 4.1 ,1)

debugMission = False

def StartMissionCallback(data):
    if debugMission : print("start arrive")
    PointToArrive = Point(data.position.x, data.position.y, data.position.z, data.orientation.z)
    ExecuteMission(PointToArrive)

rospy.Subscriber("/ArriveBallController/Start", Pose, StartMissionCallback)

ArriveBallFinishPub = rospy.Publisher('/ArriveBallController/Finish', Bool, queue_size=10)
TrajectoryPublisher = rospy.Publisher("/red/tracker/input_trajectory", MultiDOFJointTrajectory , queue_size=10)
magnetPub = rospy.Publisher('/red/uav_magnet/gain', Float32 , queue_size=10)

def ExecuteMission(TagPoint):

    trajectoryArrive, initialPointArrive = GetArriveTrajectory(TagPoint)
    
    if debugMission : print("move")
    MoveToPoint(initialPointArrive)

    msg = Float32()
    msg.data = 0.14
    magnetPub.publish(msg)

    if debugMission : print("go")
    TrajectoryPublisher.publish(trajectoryArrive.GetTrajectoryMessage())

    rate = rospy.Rate(10)
    
    rate.sleep()

    msgFinal = Bool()
    msgFinal.data = True
    ArriveBallFinishPub.publish(msgFinal)

def GetArriveTrajectory(point):

    trajectoryMission = Trajectory()

    wallDirection = 0
    inicial_x = arrive_x = point.x
    inicial_y = arrive_y = point.y
    inicial_z = arrive_z = point.z

    if point.yaw > 0.5 : wallDirection = 1.0 # left
    elif point.yaw < -0.5 : wallDirection = -1.0 # right
    else : wallDirection = 0.0 # front

    #Move Drone to 
    if wallDirection == 0: 
        inicial_x = point.x - 6.5
        arrive_x = point.x + 2.5

        inicial_y = arrive_y = point.y

        # limitação para lançamento em pontos na extremidade
        if point.y > 6.9 or point.y < -6.9:  inicial_y = arrive_y = 6.9

    else:
        inicial_x = arrive_x = point.x

        inicial_y = point.y - 6.5 * wallDirection
        arrive_y = point.y + 2.5 * wallDirection
        
        # limitação para lançamento em pontos na extremidade
        if point.x > 11.9: inicial_x = arrive_x = 11.9

    if point.z <= 3.4:
        inicial_z = point.z + 1
        arrive_z = point.z + 1.5
    elif point.z > 3.4 and point.z <= 4:
        inicial_z = point.z 
        arrive_z = point.z + 1.5
    else:
        inicial_z = 4
        arrive_z = 5.5

    PointInitial = Point(inicial_x, inicial_y, inicial_z, wallDirection)
    PointArrive = Point(arrive_x, arrive_y, arrive_z, wallDirection)
    PointFinal = Point(inicial_x, inicial_y, inicial_z, wallDirection)

    trajectoryMission.AddPointToTrajectory(PointInitial)
    trajectoryMission.AddPointToTrajectory(PointArrive)
    trajectoryMission.AddPointToTrajectory(PointFinal)

    return  ( trajectoryMission, PointInitial )


if __name__ == "__main__":
    rospy.init_node('ArriveBallController', anonymous=True)

    #ExecuteMission(PointToArriveTest)
    rospy.spin()