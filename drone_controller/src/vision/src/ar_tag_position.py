import rospy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
import cv2
import numpy as np

#conversor de imagens ros para opencv
bridge = CvBridge()

def flood_fill(img,pos,color):
    image = img.copy()
    height, width = image.shape[:2]
    mask = np.zeros((height+2, width+2), np.uint8)
    cv2.floodFill(image,mask,pos,color)
    return image

def ignore_helixes(img):
    image = img.copy()
    for row_idx,row in enumerate(image):
        brk1 = 0
        brk2 = 0
        if row[0] != 0:
            image = flood_fill(image,(0,row_idx),0)
            brk1 = 1
        idx = len(row)-1
        if row[idx] != 0:
            image = flood_fill(image,(idx,row_idx),0)
            brk2 = 1
        if brk1 == 1 and brk2 == 1:
            break
    return image

def rect_to_points(rect):
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    return box

def list_to_Vector3(input_list):
    msg = Vector3()
    msg.x = input_list[0]
    msg.y = input_list[1]
    msg.z = input_list[2]
    return msg

def calculate_artag_position(img,rect):
    global REAL_LIFE_ARTAG_WIDTH
    global gps

    image_cpy = img.copy()

    rect_x = rect[0][0]
    rect_y = rect[0][1]
    rect_width = rect[1][0]
    rect_height = rect[1][1]

    real_depth = REAL_LIFE_ARTAG_WIDTH*fx/rect_width

    centroid = (rect_x+rect_width/2,rect_y+rect_height/2)
    img_height,img_width = image_cpy.shape[:2]
    image_center = (img_width/2,img_height/2)

    image_shift_x = centroid[0] - image_center[0]
    image_shift_y = centroid[1] - image_center[1]

    real_shift_y = image_shift_x * real_depth / fx
    real_shift_z = image_shift_y * real_depth / fy

    predicted_location = [real_depth + gps.x,gps.y + real_shift_y ,gps.z + (real_shift_z)*(-1)]

    return predicted_location

#Recebe a imagem da camera e retorna a bounding box do artag
def detect_codev2(image):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    _,thresh = cv2.threshold(gray,100,255,cv2.THRESH_BINARY_INV)

    ff = flood_fill(thresh,(0,0),255)

    inv =  cv2.bitwise_not(ff)

    only_filled_shapes = inv | thresh

    result = ignore_helixes(only_filled_shapes)

    contours,_ = cv2.findContours(result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours: return

    rect = cv2.minAreaRect(contours[0])

    return rect

#Definindo as funçoes de callback
gps = None
def gps_callback(data):
    global gps
    gps = data.pose.pose.position

image = None
def camera_image_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, 'rgb8')  

#Crinado nó
node_name = 'artag_position_estimator'
rospy.init_node(node_name)
print(f'Node {node_name} iniciado com sucesso')

#Criando subsscribers
rospy.Subscriber('/red/camera/color/image_raw', Image, callback=camera_image_callback)
rospy.Subscriber('/red/odometry', Odometry, callback=gps_callback)

#Criando publishers
pub = rospy.Publisher('/artag/estimated_location', Vector3, queue_size=10)

#GLOBALS
camera_info = rospy.wait_for_message('/red/camera/color/camera_info',CameraInfo)
camera_matrix = camera_info.K
fx = camera_matrix[0]
fy = camera_matrix[4]

REAL_LIFE_ARTAG_WIDTH = 0.2

#MAIN LOOP
while(not rospy.is_shutdown()):

    image_cpy = image.copy()

    rect = detect_codev2(image_cpy)

    if not rect: continue

    box = rect_to_points(rect)

    cv2.drawContours(image,[box],0,(0,0,255),1)
    
    predicted_location = calculate_artag_position(image_cpy, rect)

    msg = list_to_Vector3(predicted_location)

    pub.publish(msg)