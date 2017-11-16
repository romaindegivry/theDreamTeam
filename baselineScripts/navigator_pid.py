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

from class_pid import PID_controller

global height, integratedY, integratedX

height = 0
integratedX = 0
integratedY = 0


global phase, target_pose

phase = 0 # Flight Phase: takeoff (0), cruise (1), landing (2)

target_pose = [[0.0,0.0,1.5],
               [6.0,0.0,1.5],
               [6.0,0.0,0.0],
               [6.0,0.0,0.0]] # End Target position of various flight phase, (meter)


global hover_phase, hover_count

hover_phase = 0 # Hover Phase: boolean (0 - not hovering / 1 - hovering)
hover_count = 0


# -- Callback Functions --


# Callback for hrlv_ez4_pub (Height Sensor)
def heightCheck(msg):

    global height    #import global range
    height = msg.range #set range = recieved range


# Callback for pose
def displacementCheck(msg):

    global integratedY
    global integratedX
    
    integratedY = msg.pose.position.y
    integratedX = msg.pose.position.x


# -- setPose Conditionals --


def getTargetPose():

    global target_pose, phase
    
    return target_pose[phase]
    

def phaseConversion(target_vel):
    
    global phase, hover_phase, hover_count, target_pose 
    
    if phase == 3:
        print('End State Achieved')
        rospy.sleep(10)
        
    
    elif target_vel[0] == 0 and target_vel[2] == 0:
        print('Entering Hover Phase.')
        hover_phase = 1
        hover_count += 1
        
        if hover_count == 20:
            print('Switching Phase {} to Phase {}'.format(phase,phase+1))
            phase += 1
            hover_count = 0
            hover_phase = 0
        
    else:
        pass


# -- setVel Algorithm --


# Simple bangBang Algorithm
def bangBang(target_disp, absTol):

    global height
    global integratedX
    global integratedY
    global phase
    
    
    # X-vel Control
    if integratedX < target_disp[0] - absTol:
        xvel = 0.3
    elif integratedX > target_disp[0] + absTol:
        xvel = -0.3
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
    
    print('X: {}, Y: {}, Z: {}'.format(integratedX, integratedY, height))
    
    
    return target_vel


# Gain for Drift Correction
def simpleGain(error, gain=1):

    return gain * error


# -- Main --


def main():

    global height, integratedX, integratedY # import global variables
    
    
    rospy.init_node('navigator')   # make ros node

    rate = rospy.Rate(20) # rate will update publisher at 20hz, higher than the 2hz minimum before tieouts occur
    stateManagerInstance = stateManager(rate) # create new statemanager


    # Subscriptions
    rospy.Subscriber("/mavros/state", State, stateManagerInstance.stateUpdate)  #get autopilot state including arm state, connection status and mode
    rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, heightCheck)  #get current distance from ground
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, displacementCheck) #subscribe to position messages
    # rospy.Subscriber("/mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad, heightCheck)
    

    # Publishers
    velPub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2) ###Change to atti
    
    
    # Controller
    controller = velControl(velPub) #create new controller class and pass in publisher and state manager
    stateManagerInstance.waitForPilotConnection()   #wait for connection to flight controller
    
    # Instantiate PID Controller
    pid_z = PID_controller(20,1,0.1,0.1)
    

    # Control Loop
    while not rospy.is_shutdown():
    
        target_pose = getTargetPose() # target displacement
        target_vel = bangBang(target_pose,0.15) # get target velocity
        controller.setVel(target_vel)
        phaseConversion(target_vel)
        
        
        controller.publishTargetPose(stateManagerInstance)
        stateManagerInstance.incrementLoop()
        rate.sleep()

        if stateManagerInstance.getLoopCount() > 100:   #need to send some position data before we can switch to offboard mode otherwise offboard is rejected
            global height
            print(pid_z(height,target_pose[2]))
            stateManagerInstance.offboardRequest()  #request control from external computer
            stateManagerInstance.armRequest()   #arming must take place after offboard is requested
            
            
    rospy.spin()    #keeps python from exiting until this node is stopped


if __name__ == '__main__':
    main()
