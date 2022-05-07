import rospy

from Class import Point 
from Functions import ( MoveToPoint)
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

def TagDetectionFinishCallback(data):
    if debugMission : print("arrive start")
    startArriveBallPub.publish(data)

def ArriveBallFinishCallback(data):
    if debugMission : print("finish")

startinPhase3 = True
debugMission = False

rospy.Subscriber("/TagDetectionController/Finish", Pose, TagDetectionFinishCallback)
rospy.Subscriber("/ArriveBallController/Finish", Bool, ArriveBallFinishCallback)

startTagDetectionPub = rospy.Publisher('/TagDetectionController/Start', Bool, queue_size=10)
startArriveBallPub = rospy.Publisher('/ArriveBallController/Start', Pose, queue_size=10)

if __name__ == "__main__":

    rospy.init_node('Task3Controller', anonymous=True)

    if(startinPhase3): 
        MoveToPoint(Point(2,0,2,0))
        if debugMission : print("move")
    
    msgTagDetection = Bool()
    msgTagDetection.data = True
    startTagDetectionPub.publish(msgTagDetection)
    if debugMission : print("detection start")

    rospy.spin()