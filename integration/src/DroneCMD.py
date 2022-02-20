# DroneCMD.py
# Esse arquivo apresenta a classe DroneCMD que contem todas as funçõs de manipulação básica do movimento do drone

import rospy
import time

from uav_ros_msgs.srv import TakeOff, TakeOffRequest
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import SetBool

from utils.MavrosState import MavrosState


class DroneCMD():

    def __init__(self) -> None:
        pass

    def takeoff(self, height:float=5) -> None:

        services = [
            "/red/mavros/cmd/arming 1",
            "/red/mavros/set_mode 0 offboard",
            "/red/takeoff", # /uav1/control_manager/takeoff
        ]

        mavros = MavrosState()

        #Armar o drone
        while not mavros.armed:
            serviceName = services[0].split(" ")[0]
            # print("Waiting for {}\n".format(serviceName))
            rospy.wait_for_service(serviceName)
            try:
                run_service = rospy.ServiceProxy(serviceName, CommandBool)
                # print("Running {}\n\n".format(serviceName))
                resp1 = run_service(1)
            except rospy.ServiceException as e:
                print("Error: %s" % e)
            time.sleep(1)

        
        #Colocar em modo offboard 
        while mavros.mode != "OFFBOARD":
            serviceName = services[1].split(" ")[0]
            # print("Waiting for {}\n".format(serviceName))
            rospy.wait_for_service(serviceName)
            try:
                run_service = rospy.ServiceProxy(serviceName, SetMode)
                # print("Running {}\n\n".format(serviceName))
                resp1 = run_service(0, "offboard")
            except rospy.ServiceException as e:
                print("Error: %s" % e)
            time.sleep(1)


        #Realizar o takeoff 
        serviceName = services[2].split(" ")[0]
        # print("Waiting for {}\n".format(serviceName))
        rospy.wait_for_service(serviceName)
        try:
            run_service = rospy.ServiceProxy(serviceName, TakeOff)
            # print("Running {}\n\n".format(serviceName))
            resp1 = run_service(height)
        
        except rospy.ServiceException as e:
            print("Error: %s" % e)
        
        # print("taking off...")

    def land(self) -> None:
        services = [
            "/red/land",
            "/red/mavros/cmd/arming 0",
        ]

        # Land
        serviceName = services[0].split(" ")[0]
        # print("Waiting for {}\n".format(serviceName))
        rospy.wait_for_service(serviceName)
        try:
            run_service = rospy.ServiceProxy(serviceName, SetBool)
            # print("Running {}\n\n".format(serviceName))
            resp1 = run_service(True)
        except rospy.ServiceException as e:
            print("Error: %s" % e)
        time.sleep(1)

        # Arming
        serviceName = services[1].split(" ")[0]
        print("Waiting for {}\n".format(serviceName))
        rospy.wait_for_service(serviceName)
        try:
            run_service = rospy.ServiceProxy(serviceName, CommandBool)
            # print("Running {}\n\n".format(serviceName))
            resp1 = run_service(0)
        except rospy.ServiceException as e:
            print("Error: %s" % e)
