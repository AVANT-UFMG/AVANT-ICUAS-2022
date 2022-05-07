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

def draw_point(point,clr):
    global image_cpy
    x = int(point[0])
    y = int(point[1])
    cv2.circle(image_cpy, (x,y), radius=1, color=clr, thickness=3)

def calculate_artag_position(img,rect):
    global REAL_LIFE_ARTAG_WIDTH
    global gps

    image_cpy = img.copy()

    rect_x = rect[0][0]
    rect_y = rect[0][1]
    rect_width = rect[1][0]
    rect_height = rect[1][1]

    real_depth = REAL_LIFE_ARTAG_WIDTH*fx/rect_width #1

    #real_depth = 12.5 - gps.x #2

    centroid = (rect_x,rect_y)

    draw_point(centroid,(255,0,0))

    img_height,img_width = image_cpy.shape[:2]
    image_center = (img_width/2,img_height/2)

    draw_point(image_center,(0,255,0))

    #Assumindo um eixo de coordenadas cartesiano centrado na imagem, calculamos os deslocamentos do artag em relação ao centro da imagem
    image_shift_x = centroid[0] - image_center[0]
    image_shift_y = image_center[1] - centroid[1]

    #Assumindo agora um eixo de coordenadas centrado no drone que segue as mesmas direções do eixo do gazebo
    real_shift_y = (-1) * image_shift_x * real_depth / fx
    real_shift_z = image_shift_y * real_depth / fy

    #Transformando para o eixo de coordenada do gazebo
    predicted_location = [gps.x + real_depth , gps.y + real_shift_y , gps.z + real_shift_z]

    print(f'\nReal depth = {real_depth}\nImage shift = {(image_shift_x,image_shift_y)}\nReal shift = {(real_shift_y,real_shift_z)}\nPredicted location = {predicted_location}')

    return predicted_location

#Recebe a imagem da camera e retorna a bounding box do artag
def detect_code(image):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    _,thresh = cv2.threshold(gray,100,255,cv2.THRESH_BINARY_INV)

    ff = flood_fill(thresh,(0,0),255)

    inv =  cv2.bitwise_not(ff)

    only_filled_shapes = inv | thresh

    cv2.imshow("detection", only_filled_shapes)
    cv2.waitKey(1)

    result = ignore_helixes(only_filled_shapes)

    contours,_ = cv2.findContours(result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours: return None

    max_cont = contours[0]
    for c in contours:
        if cv2.arcLength(c, True) > cv2.arcLength(max_cont, True):
            max_cont = c

    rect = cv2.minAreaRect(max_cont)

    return rect

#Definindo as funçoes de callback
gps = None
def gps_callback(data):
    global gps
    gps = data.pose.pose.position

image = []
def camera_image_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, 'rgb8')  

depth_map = []
def camera_depth_callback(data):
    global depth_map
    data.encoding = "mono16"
    depth_map = bridge.imgmsg_to_cv2(data, 'mono16')  

#Crinado nó
node_name = 'artag_position_estimator'
rospy.init_node(node_name)
print(f'Node {node_name} iniciado com sucesso')

#Criando subsscribers
rospy.Subscriber('/red/camera/color/image_raw', Image, callback=camera_image_callback)
rospy.Subscriber('/red/odometry', Odometry, callback=gps_callback)
rospy.Subscriber('/red/camera/depth/image_raw', Image, callback=camera_depth_callback)


#Criando publishers
location_pub = rospy.Publisher('/artag/estimated_location', Vector3, queue_size=10)
image_pub = rospy.Publisher('/red/camera/color/image_raw/artag', Image, queue_size=10000)

#GLOBALS
camera_info = rospy.wait_for_message('/red/camera/color/camera_info',CameraInfo)
camera_matrix = camera_info.K
fx = camera_matrix[0]
fy = camera_matrix[4]

REAL_LIFE_ARTAG_WIDTH = 0.2

#MAIN LOOP
while(not rospy.is_shutdown()):

    if len(image) == 0 or gps == None: continue

    image_cpy = image.copy()

    rect = detect_code(image_cpy)

    if rect == None: continue

    w = rect[1][0]

    if w <= 0: continue

    box = rect_to_points(rect)    
    
    predicted_location = calculate_artag_position(image_cpy, rect)

    msg = list_to_Vector3(predicted_location)

    location_pub.publish(msg)

    cv2.drawContours(image,[box],0,(0,0,255),1)
    cv2.imshow("cam", image_cpy)
    cv2.waitKey(1)

# while(not rospy.is_shutdown()):

#     if len(image) == 0 or gps == None: continue

#     image_cpy = image.copy()

#     rect = detect_code(image_cpy)

#     if rect == None: continue

#     box = rect_to_points(rect)    

#     cv2.drawContours(image_cpy,[box],0,(0,0,255),1)
#     cv2.imshow("cam", image_cpy)
#     cv2.waitKey(1)

#     image_pub.publish(bridge.cv2_to_imgmsg(image_cpy, encoding="passthrough"))