#!/usr/bin/env python

#import statements:
import rospy
import math
import sys
import time
from mavros_msgs.msg import OpticalFlowRad #import optical flow message structure
from mavros_msgs.msg import State  #import state message structure
from sensor_msgs.msg import Range  #import range message structure
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose #import position message structures
from geometry_msgs.msg import TwistStamped #used to set velocity messages
from mavros_msgs.srv import *   #import for arm and flight mode setting

from rosTools import * #velocity controlers + statemanagers

global height, integratedY
integratedY=0
height = 0
def distanceCheck(msg):
    global range    #import global range
    print(msg.range) #for debugging
    range = msg.range #set range = recieved range


def heightCheck(msg):
    global height
    global integratedY #import global Y drift variable
    height = msg.distance
    integratedY = msg.integrated_y

def bangBang(control,target, absTol,stateManagerInstance):
    global heightx
    global integratedY
    zvel = 0
    xvel = 0.3
    yvel = simpleGain(integratedY, -1)
    if height < target - absTol:
        zvel = 0.5
	xvel = 0.
    elif height > target + absTol:
        zvel = -0.5
    else:
	pass
    control.setVel([xvel,yvel,zvel])
    control.publishTargetPose(stateManagerInstance)

def simpleGain(error,Gain=1):
   return Gain*error

def main():
    rospy.init_node('navigator')   # make ros node

    rate = rospy.Rate(20) # rate will update publisher at 20hz, higher than the 2hz minimum before tieouts occur
    stateManagerInstance = stateManager(rate) #create new statemanager

    #Subscriptions
    rospy.Subscriber("/mavros/state", State, stateManagerInstance.stateUpdate)  #get autopilot state including arm state, connection status and mode
    global range, height, integratedY #import global variables
    rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, distanceCheck)  #get current distance from ground 
    rospy.Subscriber("/mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad,heightCheck)  #subscribe to position messages


    #Publishers
    velPub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2) ###Change to atti


    controller = velControl(velPub) #create new controller class and pass in publisher and state manager
    stateManagerInstance.waitForPilotConnection()   #wait for connection to flight controller





    while not rospy.is_shutdown():
	bangBang(controller,1.5,0.2,stateManagerInstance)
        stateManagerInstance.incrementLoop()
        rate.sleep()    #sleep at the set rate
        if stateManagerInstance.getLoopCount() > 100:   #need to send some position data before we can switch to offboard mode otherwise offboard is rejected
            stateManagerInstance.offboardRequest()  #request control from external computer
            stateManagerInstance.armRequest()   #arming must take place after offboard is requested
    rospy.spin()    #keeps python from exiting until this node is stopped


if __name__ == '__main__':
    main()



