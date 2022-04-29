import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

bridge = CvBridge()

def IsBlackColor(pointRGB):
    return pointRGB[0] < 40 and pointRGB[1] < 50 and pointRGB[2] < 50

def FilterColorBlack(image):
    hlsImage = cv.cvtColor(image, cv.COLOR_RGB2HLS)
    hsvImage = cv.cvtColor(image, cv.COLOR_RGB2HSV)

    h,s,v = cv.split(hsvImage)

    for x in range(0, len(hlsImage)):
        for y in range(0, len(hlsImage[0])):
            if ( hlsImage[x, y][2] < 1 ) :
                hlsImage[x, y][2] = 100
                hlsImage[x, y][0] = 50
                hlsImage[x, y][1] = 100
                #hlsImage[x, y][2] *= 0.5
            else :
                hlsImage[x, y][2] = 0
                hlsImage[x, y][0] = 50
                hlsImage[x, y][1] = 0

    return cv.cvtColor(hlsImage, cv.COLOR_HLS2RGB)

def filterColorWhite(image):
    hsvImage = cv.cvtColor(image, cv.COLOR_RGB2HLS)
    h, l, s = cv.split(hsvImage)

    for x in range(0, len(hsvImage)):
        for y in range(0, len(hsvImage[0])):
            if hsvImage[x, y][2] > 99 :
                hsvImage[x, y][2] = 100
                hsvImage[x, y][0] = 50
                hsvImage[x, y][1] = 100
                #hsvImage[x, y][2] *= 0.5
            else :
                hsvImage[x, y][2] = 0
                hsvImage[x, y][0] = 50
                hsvImage[x, y][1] = 0

    return cv.cvtColor(hsvImage, cv.COLOR_HLS2RGB)

def cutandresizeImage(image):
    height, width = image.shape[:2]
    resizeImages = list()

    image1 = image[0:int(height/2) , 0:int(width/2)]
    image2 = image[0:int(height/2) , int(width/2):width]
    image3 = image[int(height/2):height , 0:int(width/2)]
    image4 = image[int(height/2):height  , int(width/2):width]
    image5 = image[int(height/4)-50:int(3*height/4)+50 , int(width/4)-50:int(3*width/4)+50]

    #image1,image2,image3,image4,
    imagesCrop = [image5]

    for i in imagesCrop:
        height, width = i.shape[:2]

        resized_up = cv.resize(i, (height*2, width*2), interpolation= cv.INTER_LINEAR)
        resizeImages.append(resized_up)

    return resizeImages

ImageCallback = None
def CameraCallback(data):
    global ImageCallback
    ImageCallback = bridge.imgmsg_to_cv2(data, 'bgr8') 
    #print("receive")

def GetImageFromDrone():
    return ImageCallback

def LabelImage(squares, image):
    imageLabelTag =  image.copy()

    cv.drawContours(imageLabelTag, squares, -1, (255, 255, 0), 1)
    #cv.circle(imageLabelTag, (xCenter, yCenter), 1, (255, 0, 255), -1)

    return imageLabelTag

def GetImageToPublish(img):
    return bridge.cv2_to_imgmsg(img)

