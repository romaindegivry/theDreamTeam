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
import numpy as np
"""
This is the navigator file we use for submission

The drone's estimated state is held in a global dictionary
The node's cofiguration parameters are held in another dictionnary
"""
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)#setting up the logs
fh = logging.FileHandler('log_filename.txt')
fh.setLevel(logging.DEBUG)
logger.addHandler(fh)

ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
logger.addHandler(ch)

if __name__ == "__main__":
    #controls the behaviour when called from the command line
    if len(sys.argv) == 3:
        TARGET = [float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])]
    else:
        pass
    logger.debug('{} arguments processed'.format(len(sys.argv)-1))


#State vector (update using estimateState)
global droneState
droneState = {'pos'         :   np.array([0.,0.,0.]),#initialize postion
              'quaternion'  :   np.array([0.,0.,0.,0.]),#initialize attitude
              'velLinear'   :   np.array([0.,0.,0.]),#initialize velocity
              'velAngular'  :   np.array([0.,0.,0.])}    #initialize angular velocity

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
sensorState = {'linearPose' :   np.array([0.,0.,0.]),
               'quatPose'   :   np.array([0.,0.,0.,0.]),
               'linearVel'  :   np.array([0.,0.,0.]),
               'angularVel ':   np.array([0.,0.,0.]),
               'linearAcc'  :   np.array([0.,0.,0.]),
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
    sensorState['linearPose'] = np.array([msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z])
    #Figures out the roation
    sensorState['quatPose'] =  np.array([msg.pose.orientation.x,
                                msg.pose.orientation.y,
                                msg.pose.orientation.z,
                                msg.pose.orientation.w])
    #get the time of the clock
    clockState['pose'] = msg.header.stamp

def velCheck(msg):
    """
    This function recovers the velocity estimation from the pixHawk
    It updates the sensor state automatically
    """
    global clockState, sensorState
    #get cartesian velocity
    sensorState['linearVel'] = np.array([msg.twist.linear.x,
                                msg.twist.linear.y,
                                msg.twist.linear.z])
    #get angular velocity
    sensorState['angularVel'] = np.array([msg.twist.angular.x,
                                 msg.twist.angular.y,
                                 msg.twist.angular.z])
    #update clock time
    clockState['vel'] = msg.header.stamp
    
def imuCheck(msg):

    global clockState, sensorState
    
    sensorState['linearAcc'] = np.array([msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z])
                               
    clockState['IMU'] = msg.header.stamp
    

#main program
def main(logger):
    global droneState, clockState, nodeState, sensorState
    logger.debug("Begun main loop")
    
    #starting the logfile writer
    if nodeState['logMode'] == True:
        logging.debug("Begun logging position data")
        csvfile = open(nodeState['logFile'].format(hash(time.clock())%1000), 'w')    
        writer = csv.DictWriter(csvfile, fieldnames = fieldnames)
        writer.writeheader()
       
    else:
        #unless we don't want logs
        csvfile = None
        logger.info('No logs will be taken')
   
   
   
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
    
    #Initialize the mission manager:
    manager = helpers.MissionManager(lambda _: rospy.signal_shutdown("Mission End"))
    
    PID = PID_controller(nodeState['rate'],k_p = [-0.04,-0.04,-0.04],k_i = [-0.01,-0.01,-0.01],maxVel = 0.5,minVel = 0.0)
    
    
    flight = helpers.FlightManager('ramp',1.5,np.array([0.0,0.,1.5]),**nodeState)
    flight.setController(PID)
    manager.addSegment(flight)
    
    stateManagerInstance.waitForPilotConnection()   #wait for connection to flight controller
    logger.debug("Started mission routine - entering main loop")
    while not rospy.is_shutdown():
        controller.publishTargetPose(stateManagerInstance)
        stateManagerInstance.incrementLoop()
        rate.sleep()
        
        #main body of the program
        if stateManagerInstance.getLoopCount() > 100:
            #Apply calibration
            helpers.addDicts(sensorState,initialSensorState)
            
            #generate new phase
            manager.update(droneState,clockState,**nodeState)
            print(manager.phase().name)
            #generate control input
            ctrl = manager.controller()
            ctrl.update(droneState,clockState,manager.phase().target)
            
            controller.setVel(manager.controller()()) #send input [vx,vy,vz]

            #update general state
            helpers.updateState(droneState,sensorState,clockState)
            #log system state
            
            #writer.writerow(sensorState)
            stateManagerInstance.offboardRequest()  #request control from external computer
            stateManagerInstance.armRequest() #arming must take place after offboard is requested
            
        #Special case for sensor and clock initialisation
        elif stateManagerInstance.getLoopCount() == 100:
            logger.info("Calibrating sensors")
            initialSensorState = helpers.negateDict(sensorState)
            PID.reset(clockState)#reset the PID
            logger.info("Releasing control to mission state manager")
            time.sleep(1)
            
        else:
            pass
     
     
     
     #perform the cleanup
    if csvfile != None: #close log file
        csvfile.close()
        
    logger.debug("cleanup complete, process is ready to shutdown")
    
    

    rospy.spin()    #keeps python from exiting until this node is stopped
if __name__=="__main__":
    main(logger)
    
