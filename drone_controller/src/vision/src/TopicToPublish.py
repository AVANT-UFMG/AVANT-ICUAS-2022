import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point 


##node to simulate the final result 

def callbackPoint(data):
    print("sendPoint")

def callbackImage(data):
    print("sendImage")

if __name__ == "__main__":
    rospy.init_node('Topic', anonymous=True)

    rospy.Subscriber("/red/tag_position_reconstructed", Point, callbackPoint)
    rospy.Subscriber("/red/tag_image_annotated", Image, callbackImage)

    rospy.spin()
