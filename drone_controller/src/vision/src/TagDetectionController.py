import rospy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

from Class import Point
from Functions import (GetStrategies, MoveToPoint, ArrivePoint, 
                        GetImageFromDrone, GetArTag, EstimatePointPosition,
                        GetDronePoint, LabelImage, GetImageToPublish, WaitStablePosition)



debugMission = False

def StartMissionCallback(data):
    if debugMission : print("start Mission")
    ExecuteMission()

rospy.Subscriber("/TagDetectionController/Start", Bool, StartMissionCallback)

PointEstimatedPub = rospy.Publisher('/red/tag_position_reconstructed', PointMsg, queue_size=10)
ImageAnnotatedPub = rospy.Publisher('/red/tag_image_annotated', Image, queue_size=10000)
TagDetectionFinishPub = rospy.Publisher('/TagDetectionController/Finish', Pose, queue_size=10)

def ExecuteMission():
    # TODO CheckDroneInArea3()

    # Go to Point to Detect()
    findTag = False
    ImagePoint = None
    xCenter = 0
    yCenter = 0 

    strategies = GetStrategies()

    for strategie in strategies:
        if findTag : break

        # The first message is not execute, need analise    
        # PublishDeadPoint()
        if debugMission : print("start")

        for point in strategie:
            squares = []

            ImagePoint = None
            xCenter = 0
            yCenter = 0 
        
            MoveToPoint(point)
            if debugMission : print("move")

            ArrivePoint(point)
            if debugMission : print("arrive")

            WaitStablePosition()
            if debugMission : print("stable")
            
            # Scan area to find target 

            while  ImagePoint is  None:
                ImagePoint  = GetImageFromDrone()
                if debugMission : print("wait Image")

            squares, xCenter, yCenter = GetArTag(ImagePoint, 0.8, 5, 10, 80, 0.1)

            if debugMission : print(squares, xCenter, yCenter)

            if len(squares) > 0 : 
                if debugMission : print("find")
                findTag = True
                break
            
    # Find point in World
    currentPointDrone = GetDronePoint()

    if debugMission : print(xCenter, yCenter,currentPointDrone )
    resultWorldPoint  = EstimatePointPosition(ImagePoint, xCenter, yCenter, currentPointDrone)
    finalImageLabel = LabelImage(squares, ImagePoint)

    if debugMission : print("finish")

    # Publish Image
    if debugMission : print(resultWorldPoint)

    msgFinalController = Pose()
    msgFinalController.position.x = resultWorldPoint.x
    msgFinalController.position.y = resultWorldPoint.y
    msgFinalController.position.z = resultWorldPoint.z
    msgFinalController.orientation.z = currentPointDrone.orientation.z
    msgFinalController.orientation.w = 1.0

    msgFinal = PointMsg()
    msgFinal.x = resultWorldPoint.x
    msgFinal.y = resultWorldPoint.y
    msgFinal.z = resultWorldPoint.z

    TagDetectionFinishPub.publish(msgFinalController)
    PointEstimatedPub.publish(msgFinal)
    ImageAnnotatedPub.publish(GetImageToPublish(finalImageLabel))

if __name__ == "__main__":
    rospy.init_node('TagDetectionController', anonymous=True)

    #ExecuteMission()
    rospy.spin()
    
    