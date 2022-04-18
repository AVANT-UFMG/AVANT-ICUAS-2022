import math
 
def euler_from_quaternion(x, y, z, w):

    # Convert a quaternion into euler angles (roll, pitch, yaw)
    # roll is rotation around x in radians (counterclockwise)
    # pitch is rotation around y in radians (counterclockwise)
    # yaw is rotation around z in radians (counterclockwise)

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def estimate_point_position(image, in_x, in_y, pose):

    # This function inputs are an image, x and y coordinates and a Pose.
    # It returns a 3D point which corresponds to the global coordinates of a specified 2D point in an image captured by the UAV.

    CAMERA_FOCAL_DISTANCE = 381.36246688113556

    img_height,img_width = image.shape[:2]
    image_center = (img_width/2,img_height/2)

    #Assuming a cartesian coordinate system centered on the image, we calculate the point displacements with respect to the center of the image
    image_shift_x = in_x - image_center[0]
    image_shift_y = image_center[1] - in_y

    x,y,z,w = pose.orientation
    roll_x, pitch_y, yaw_z = euler_from_quaternion(x,y,z,w)

    #Positive X
    if math.pi * 1/4 <= yaw_z < math.pi * 3/4:
        distance_to_wall = 12.5 - pose.position.x
        horizontal_component = image_shift_x * distance_to_wall / CAMERA_FOCAL_DISTANCE
        vertical_component = image_shift_y * distance_to_wall / CAMERA_FOCAL_DISTANCE
        predicted_location = [pose.position.x + distance_to_wall , pose.position.y - horizontal_component , pose.position.z + vertical_component]
    
    #Positive Y
    elif math.pi * 3/4 <= yaw_z < math.pi * 5/4:
        distance_to_wall = 7.5 - pose.position.
        horizontal_component = image_shift_x * distance_to_wall / CAMERA_FOCAL_DISTANCE
        vertical_component = image_shift_y * distance_to_wall / CAMERA_FOCAL_DISTANCE
        predicted_location = [pose.position.x + horizontal_component , pose.position.y + distance_to_wall, pose.position.z + vertical_component]

    #Negative X
    elif math.pi * 5/4 <= yaw_z < math.pi * 7/4:
        distance_to_wall = pose.position.x - 12.
        horizontal_component = image_shift_x * distance_to_wall / CAMERA_FOCAL_DISTANCE
        vertical_component = image_shift_y * distance_to_wall / CAMERA_FOCAL_DISTANCE
        predicted_location = [pose.position.x - distance_to_wall , pose.position.y + horizontal_component, pose.position.z + vertical_component]

    #Negative Y
    else:
        distance_to_wall = pose.position.y - 7.5
        horizontal_component = image_shift_x * distance_to_wall / CAMERA_FOCAL_DISTANCE
        vertical_component = image_shift_y * distance_to_wall / CAMERA_FOCAL_DISTANCE
        predicted_location = [pose.position.x - horizontal_component , pose.position.y - distance_to_wall, pose.position.z + vertical_component]
    
    return predicted_location