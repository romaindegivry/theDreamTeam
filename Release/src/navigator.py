#!/usr/bin/env python
#coding: utf-8

#import (stdlib)
import math
import sys
import time
import csv
import logging

#import (local)
from dep.rosTools import * #velocity controlers + statemanagers
from dep.class_pid import PID_controller
import dep.helpers as helpers

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
              'velAngular'  :   np.array([0.,0.,0.]),
              'height'      :   0.}    #initialize angular velocity

fieldnames = ['phase',
              'time_sonar',  
              'h_sonar',    
              'time_pose',   
              'x', 
              'y', 
              'z',  
              'quat_x', 
              'quat_y', 
              'quat_z', 
              'quat_w',    
              'time_vel',   
              'x_dot', 
              'y_dot', 
              'z_dot',  
              'ang_x_dot', 
              'ang_y_dot', 
              'ang_z_dot',   
              'time_imu', 
              'x_dot2', 
              'y_dot2', 
              'z_dot2',  
              'in_x_dot', 
              'in_y_dot', 
              'in_z_dot']
                 
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
    clockState['sonar'] = msg.header.stamp.to_sec() 

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
    clockState['pose'] = msg.header.stamp.to_sec() 

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
    clockState['vel']= msg.header.stamp.to_sec() 
 
    
def imuCheck(msg):

    global clockState, sensorState
    
    sensorState['linearAcc'] = np.array([msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z])
                               
    clockState['IMU'] = msg.header.stamp.to_sec() 
    

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

    PID = PID_controller(nodeState['rate'],k_p = [-0.4,-0.4,-3],k_i = [0,-0.01,0],k_d = [-0.02,-0.02,0],maxVel = 0.4,minVel = 0.0)

    start = helpers.takeOffManager('takeoff',[0.0,0,1.5],**nodeState)
    tuning = helpers.tuneManager('tune',[0.,0.,1.5],**nodeState)
    ramp = helpers.FlightManager('ramp',1.5,np.array([6.0,0.,1.5]),**nodeState)
    flight = helpers.FlightManager('fly',1.5,np.array([6.0,0.,1.5]),**nodeState)
    landing = helpers.LandingManager('land',[6.0,0,0.1],**nodeState)
    
    #give some PIDs to the people!
    start.setController(PID)
    tuning.setController(PID)
    ramp.setController(PID)
    flight.setController(PID)
    landing.setController(PID)

    #adding segments to the manager
    manager.addSegment(start)
    manager.addSegment(tuning)
    manager.addSegment(ramp)
    manager.addSegment(flight)
    manager.addSegment(landing)
    
    #takeoff before anything
    manager.initialSegment('takeoff')
    
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
            #helpers.addDicts(clockState,initialclockState)
            
            #generate new phase
            manager.update(droneState,clockState,**nodeState)

            #generate control input
            ctrl = manager.controller()
            ctrl.update(droneState,clockState,manager.phase().target)
            

            controller.setVel(manager.controller()()) #send input [vx,vy,vz]

            #update general drone state
            helpers.updateState(droneState,sensorState,clockState)
            #log system state
            
            if nodeState['logMode']:
                row = helpers.logDict(manager.phase().name, 
                              droneState,clockState,nodeState,sensorState,manager.controller()())
                writer.writerow(row)
            
            
            #writer.writerow(sensorState)
            stateManagerInstance.offboardRequest()  #request control from external computer
            stateManagerInstance.armRequest() #arming must take place after offboard is requested
            
        #Special case for sensor and clock initialisation
        elif stateManagerInstance.getLoopCount() == 100:
            logger.info("Calibrating sensors")
            initialSensorState = helpers.negateDict(sensorState)
            initialclockState = helpers.negateDict(clockState)
            PID.reset(clockState)#reset the PID
            logger.info("Releasing control to mission state manager")
            time.sleep(1)
            
        else:
            pass
     
     
     
     #perform the cleanup
    if csvfile != None: #close log file
        csvfile.close()
        del writer
        
    logger.debug("cleanup complete, process is ready to shutdown")
    
    

    rospy.spin()    #keeps python from exiting until this node is stopped
if __name__=="__main__":
    main(logger)
    
