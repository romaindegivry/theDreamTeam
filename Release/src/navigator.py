#!/usr/bin/env python
#coding: utf-8

#import (stdlib)
import math
import sys
import time
import csv
import logging

#import (local)
from rosTools import * #velocity controlers + statemanagers
from class_pid import PID_controller
import helpers
#import (3rd party)
import rospy

"""
This is the navigator file we use for submission

The drone's estimated state is held in a global dictionary
The node's cofiguration parameters are held in another dictionnary
"""

logging.basicConfig(filename='navigation.log',level=logging.DEBUG)#setting up the logs

if __name__ == "__main__":
    #controls the behaviour when called from the command line
    if len(sys.argv) == 3:
        TARGET = [float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])]
    else:
        pass
    logging.debug('{} arguments processed'.format(len(sys.argv)-1))


#State vector (update using estimateState)
global droneState
droneState = {'pos'         :   [0.,0.,0.],#initialize postion
              'quaternion'  :   [0.,0.,0.,0.],#initialize attitude
              'velLinear'   :   [0.,0.,0.],#initialize velocity
              'velAngular'  :   [0.,0.,0.]}    #initialize angular velocity

#initializing time
global clockState
clockState = {'sonar'       :   0.,
              'pose'        :   0.,
              'vel'         :   0.,
              'IMU'         :   0.}

#initialize flight state
global nodeState
nodeState = {'rate'    :    20      ,#20Hz sampling rate
             'logMode' :    False    ,#logging the flight data
             'logFile' :    '../out/flightLog{}.log',#log output path
             'mode'    :    'Exam'  }#Flight mode
              
#initialize sensors
global sensorState
sensorState = {'linearPose' :   [0.,0.,0.],
               'quatPose'   :   [0.,0.,0.,0.],
               'linearVel'  :   [0.,0.,0.],
               'angularVel ':   [0.,0.,0.],
               'linearAcc'  :   [0.,0.,0.],
               'height'     :   0.,
               'flowX'      :   0.,
               'flowY'      :   0.}

##CALLBACK FUNCTIONS
def heightCheck(msg):
    """
    This function recovers the value and timestamp of the range sensor
    It updates the state automatically
    """
    global clockState, sensorState
    sensorState['height'] = msg.range #set range = recieved range
    clockState['sonar'] = msg.header.stamp

def displacementCheck(msg):
    """
    This function recovers the pose estimation from the pixhawk
    It updates the sensor state automatically
    """
    global clockState, sensorState
    
    #get cartesian position estimation
    sensorState['linearPose'] = [msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z]
    #Figures out the roation
    sensorState['quatPose'] =  [msg.pose.orientation.x,
                                msg.pose.orientation.y,
                                msg.pose.orientation.z,
                                msg.pose.orientation.w]
    #get the time of the clock
    clockState['pose'] = msg.header.stamp

def velCheck(msg):
    """
    This function recovers the velocity estimation from the pixHawk
    It updates the sensor state automatically
    """
    global clockState, sensorState
    #get cartesian velocity
    sensorState['linearVel'] = [msg.twist.linear.x,
                                msg.twist.linear.y,
                                msg.twist.linear.z]
    #get angular velocity
    sensorState['angularVel'] = [msg.twist.angular.x,
                                 msg.twist.angular.y,
                                 msg.twist.angular.z]
    #update clock time
    clockState['vel'] = msg.header.stamp
    
def imuCheck(msg):

    global clockState, sensorState
    
    sensorState['linearAcc'] = [msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z]
                               
    clockState['IMU'] = msg.header.stamp
    

#main program
def main():
    global droneState, clockState, nodeState, sensorState
    logging.debug("Begun main loop")
    
    #starting the logfile writer
    if nodeState['logMode'] == True:
        logging.debug("Begun logging position data")
        csvfile = open(nodeState['logFile'].format(hash(time.clock())%1000), 'w')    
        writer = csv.DictWriter(csvfile, fieldnames = fieldnames)
        writer.writeheader()
       
    else:
        #unless we don't want logs
        csvfile = None
        logging.info('No logs will be taken')
   
   
   
    ##START ROSPY
    # make ros node
    #Signals disabled to allow node shutdown 9at landing)
    rospy.init_node('navigator',disable_signals=True)
    
	# rate will update publisher at 20hz, higher than the 2hz minimum before tieouts occur
    rate = rospy.Rate(nodeState['rate']) 
    
    stateManagerInstance = stateManager(rate) # create new statemanager


    # Subscriptions
    #get autopilot state including arm state, connection status and mode
    rospy.Subscriber("/mavros/state", State, stateManagerInstance.stateUpdate)  
    #get current distance from ground
    #Range is a message Structure object (see rosTools.py)
    rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, heightCheck)  
    #subscribe to position messages
    #PoseStamped is a message structure object (see rosTools.py)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, displacementCheck) 
    #Optical flow sensor link
    """rospy.Subscriber("/mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad, heightCheck)"""
    # Velocity Sensor
    rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, velCheck)
    # Acceleration Sensor
    rospy.Subscriber("/mavros/imu/data", Imu, imuCheck)
    

    # Publishers
    velPub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2) 
    
    # Controller
    #create new controller class and pass in publisher and state manager
    controller = velControl(velPub)
    
    stateManagerInstance.waitForPilotConnection()   #wait for connection to flight controller
    logging.debug("Started mission routine - entering main loop")
    while not rospy.is_shutdown():
        controller.publishTargetPose(stateManagerInstance)
        stateManagerInstance.incrementLoop()
        rate.sleep()
        
        #main body of the program
        if stateManagerInstance.getLoopCount() > 100:
            #Apply clibration
            helpers.addDicts(sensorState,initialSensorState)
            
            #generate new phase
            
            #generate control input
            controller.setVel([0,0,10]) #send input [vx,vy,vz]

            #update general state
            
            #log system state
            #writer.writerow(sensorState)
            stateManagerInstance.offboardRequest()  #request control from external computer
            stateManagerInstance.armRequest() #arming must take place after offboard is requested
        #Special case for sensor anc clock initialisation
        elif stateManagerInstance.getLoopCount() == 100:
            print('hello')
            logging.info("Calibrating sensors")
            initialSensorState = helpers.negateDict(sensorState)
            logging.info("Releasing control to mission state manager")
            
        else:
            pass
     
     
     
     #perform the cleanup
    if csvfile != None: #close log file
        csvfile.close()
        
    logging.debug("cleanup complete, process is ready to shutdown")
    
    

    rospy.spin()    #keeps python from exiting until this node is stopped
if __name__=="__main__":
    main()
    
