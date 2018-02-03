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

global height, integratedY, integratedX
height = 0
integratedX = 0
integratedY = 0


def distanceCheck(msg):

    global height    #import global range
    #print(msg.range) #for debugging
    height = msg.range #set range = recieved range


def heightCheck(msg):

    global height
    height = msg.distance
    
    
def displacementCheck(msg):

    global integratedY
    global integratedX
    
    integratedY = msg.pose.position.y
    integratedX = msg.pose.position.x


def bangBang(target_disp, absTol):

    global height
    global integratedX
    global integratedY
    
    # X-vel Control
    if integratedX < target_disp[0] - absTol:
        xvel = 0.2
    elif integratedX > target_disp[0] + absTol:
        xvel = -0.2
    else:
        xvel = 0.0

    # Z-vel Control
    if height < target_disp[2] - absTol:
        zvel = 0.5
    elif height > target_disp[2] + absTol:
        zvel = -0.5
    else:
    	zvel = 0.0

    yvel = simpleGain(integratedY, -1)
    
    target_vel = [xvel,yvel,zvel]
    
    
    return target_vel


def simpleGain(error,Gain=1):

    return Gain*error


def main():

    global height, integratedX, integratedY #import global variables
    
    rospy.init_node('navigator')   # make ros node

    rate = rospy.Rate(20) # rate will update publisher at 20hz, higher than the 2hz minimum before tieouts occur
    stateManagerInstance = stateManager(rate) #create new statemanager

    #Subscriptions
    rospy.Subscriber("/mavros/state", State, stateManagerInstance.stateUpdate)  #get autopilot state including arm state, connection status and mode
    rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, distanceCheck)  #get current distance from ground
    #rospy.Subscriber("/mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad, heightCheck)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, displacementCheck) #subscribe to position messages


    #Publishers
    velPub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2) ###Change to atti

    controller = velControl(velPub) #create new controller class and pass in publisher and state manager
    stateManagerInstance.waitForPilotConnection()   #wait for connection to flight controller

    while not rospy.is_shutdown():
    
        target_vel = bangBang([6,0,3],0.2)
        controller.setVel(target_vel)
        
        
        print('X: {}, Y: {}, Z: {}'.format(integratedX, integratedY, height))
        
        controller.publishTargetPose(stateManagerInstance)
        stateManagerInstance.incrementLoop()
        rate.sleep()    #sleep at the set rate

        if stateManagerInstance.getLoopCount() > 100:   #need to send some position data before we can switch to offboard mode otherwise offboard is rejected
            stateManagerInstance.offboardRequest()  #request control from external computer
            stateManagerInstance.armRequest()   #arming must take place after offboard is requested
            
            
    rospy.spin()    #keeps python from exiting until this node is stopped


if __name__ == '__main__':
    main()
