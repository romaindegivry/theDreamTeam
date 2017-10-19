#!/usr/bin/env python

#import statements:
import rospy
import math
import sys
import time
from mavros_msgs.msg import OpticalFlowRad #import optical flow message structure
from mavros_msgs.msg import State  #import state message structure
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose #import position message structures
from mavros_msgs.srv import *   #import for arm and flight mode setting






class velControl:
    def __init__(self, attPub):  #attPub = attitude publisher
        self._attPub = attPub
        self._setVelMsg = PoseStamped()
        self._targetX = 0
        self._targetY = 0
        self._targetZ = 0

    
    def setVel(self, coordinates):
        self._targetX = float(coordinates[0])
        self._targetY = float(coordinates[1])
        self._targetZ = float(coordinates[2])
        rospy.logwarn("Target location is \nx: {} \ny: {} \nz: {}".format(self._targetX,self._targetY, self._targetZ))


    def publishTargetPose(self, stateManagerInstance):
        self._setVelMsg.header.stamp = rospy.Time.now()    #construct message to publish with time, loop count and id
        self._setVelMsg.header.seq = stateManagerInstance.getLoopCount()
        self._setVelMsg.header.frame_id = 'fcu'

        self._setVelMsg.pose.position.x = self._targetX
        self._setVelMsg.pose.position.y = self._targetY
        self._setVelMsg.pose.position.z = self._targetZ
        
        self._setVelMsg.pose.orientation.x = 0 
        self._setVelMsg.pose.orientation.y = 0 
        self._setVelMsg.pose.orientation.z = 0 
        self._setVelMsg.pose.orientation.w = 1 
        
        self._attPub.publish(self._setVelMsg) 
        
        
        
        
        
class stateManager: #class for monitoring and changing state of the controller
    def __init__(self, rate):
        self._rate = rate
        self._loopCount = 0
        self._isConnected = 0
        self._isArmed = 0
        self._mode = None
    
    def setLoopCounter(self, loop):
        self._loopCount = loop

    def getLoopCount(self):
        return self._loopCount

    def stateUpdate(self, msg):
        self._isConnected = msg.connected
        self._isArmed = msg.armed
        self._mode = msg.mode
        rospy.logwarn("Connected is {}, armed is{}, mode is {} ".format(self._isConnected, self._isArmed, self._mode)) ###debug


    def armRequest(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            modeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode) #get mode service and set to offboard control
            modeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Service mode set faild with exception: %s"%e)
    
    def offboardRequest(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool) #get arm command service and arm
            arm(True)
        except rospy.ServiceException as e:   #except if failed
            print("Service arm failed with exception :%s"%e)


    def waitForPilotConnection(self):   #wait for connection to flight controller
        rospy.logwarn("Waiting for pilot connection")
        while not rospy.is_shutdown():  #while not shutting down
            if self._isConnected:   #if state isConnected is true
                rospy.logwarn("Pilot is connected")
                return True
            self._rate.sleep
        rospy.logwarn("ROS shutdown")
        return False




def main():
    rospy.init_node('navigator')   # make ros node
    


    rate = rospy.Rate(20) # rate will update publisher at 20hz, higher than the 2hz minimum before tieouts occur
    stateManagerInstance = stateManager(rate) #create new statemanager

    #Subscriptions
    rospy.Subscriber("/mavros/state", State, stateManagerInstance.stateUpdate)  #get autopilot state including arm state, connection status and mode
    ###rospy.Subscriber("/mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad, callback)  #subscribe to position messages


    #Publishers
    velPub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=2) ###Change to atti


    controller = velControl(velPub) #create new controller class and pass in publisher and state manager
    stateManagerInstance.waitForPilotConnection()   #wait for connection to flight controller





    while not rospy.is_shutdown():
        controller.setVel([100,100,100])
        controller.publishTargetPose(stateManagerInstance)
        stateManagerInstance.setLoopCounter(stateManagerInstance.getLoopCount() + 1)
        rate.sleep()    #sleep at the set rate
        if stateManagerInstance.getLoopCount() > 100:   #need to send some position data before we can switch to offboard mode otherwise offboard is rejected
            stateManagerInstance.offboardRequest()  #request control from external computer
            stateManagerInstance.armRequest()   #arming must take place after offboard is requested
    rospy.spin()    #keeps python from exiting until this node is stopped


if __name__ == '__main__':
    main()



