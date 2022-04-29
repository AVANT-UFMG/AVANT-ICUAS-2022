import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point as PointMsg
from std_msgs.msg import Bool

from Class import Point 
from Functions import (GetStrategies, MoveToPoint, ArrivePoint, PublishDeadPoint, 
                        CameraCallback, GetImageFromDrone, GetArTag, EstimatePointPosition,
                        GetDronePoint, LabelImage, GetImageToPublish)


def StartMissionCallback(data):
    ExecuteMission()

rospy.Subscriber('/red/camera/color/image_raw', Image, callback=CameraCallback)
rospy.Subscriber("TagDetectionController", Bool, StartMissionCallback)

pointEstimatedPub = rospy.Publisher('/red/tag_position_reconstructed', PointMsg, queue_size=10)
imageAnnotatedPub = rospy.Publisher('/red/tag_image_annotated', Image, queue_size=10000)

debugMission = True
def ExecuteMission():
    # TODO CheckDroneInArea3()

    # Go to Point to Detect()
    findTag = False

    strategies = GetStrategies()

    for strategie in strategies:
        if findTag : break

        # The first message is not execute, need analise    
        PublishDeadPoint()
        if debugMission  : print("start")

        for point in strategie:
            squares = []
            xCenter = 0
            yCenter = 0
            ImagePoint = None
            
            MoveToPoint(point)
            if debugMission  : print("move")

            ArrivePoint(point)
            print("arrive")
            
            # Scan area to find target 
            while  ImagePoint is  None:
                ImagePoint  = GetImageFromDrone()

            squares, xCenter, yCenter = GetArTag(ImagePoint, 0.8, 5, 10, 80, 0.1)

            if debugMission  : print(squares, xCenter, yCenter)

            if len(squares) > 0 : 
                if debugMission  : print("find")
                findTag = True
                break
            
    # Find point in World
    currentPointDrone = GetDronePoint()
    resultWorldPoint  = EstimatePointPosition(ImagePoint, xCenter, yCenter, currentPointDrone)
    finalImageLabel = LabelImage(squares, ImagePoint)

    if debugMission  : print("finish")

    # Publish Image
    pointEstimatedPub.publish(resultWorldPoint)
    imageAnnotatedPub.publish(GetImageToPublish(finalImageLabel))

if __name__ == "__main__":

    rospy.init_node('TagDetectionController', anonymous=True)

    rospy.spin()
    
    