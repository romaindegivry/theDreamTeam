#!/usr/bin/env python

#import statements:
import rospy
import math
import sys
import time

from rosTools import * #velocity controlers + statemanagers

from class_pid import PID_controller


print("##########################START############################")
if __name__ == "__main__":
    TARGET = [float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])]
#global variables for the state of the node
global nodeState
nodeState = {'rate' : 20}

#global variables to track the state of the aircraft
# from local_position/pose
global height, pos, quat

height = 0
pos = [0.0, 0.0, 0.0]
quat = [0.0, 0.0, 0.0, 0.0]

#global variables for velocities of aircraft
# from local_position/velocity
global linear_vel, ang_vel

linear_vel = [0.0, 0.0, 0.0]
ang_vel = [0.0, 0.0, 0.0]

#Timestamps for topics
global time_sonar, time_pose, time_vel, time_imu

time_sonar = 0
time_pose = 0
time_vel = 0
time_imu = 0

#global variables for acceleration from IMU
# from imu/data
global linear_acc

linear_acc = [0.0, 0.0, 0.0]


#global variables for the state of the controller
global phase, target_pose

phase = 0 # Flight Phase: takeoff (0), cruise (1), landing (2)

target_pose = [[0.0,0.0,1.5],TARGET] # End Target position of various flight phase, (meter)


global hover_phase, hover_count

hover_phase = 0 # Hover Phase: boolean (0 - not hovering / 1 - hovering)
hover_count = 0


# -- Callback Functions --


# Callback for hrlv_ez4_pub (Height Sensor)
def heightCheck(msg):

    global height    #import global range
    global time_sonar
    
    height = msg.range #set range = recieved range
    time_sonar = msg.header.stamp


# Callback for pose
def displacementCheck(msg):
    
    global pos
    global quat
    global time_pose
    
    pos[0] = msg.pose.position.x
    pos[1] = msg.pose.position.y
    pos[2] = msg.pose.position.z
    
    quat[0] = msg.pose.orientation.x    
    quat[1] = msg.pose.orientation.y
    quat[2] = msg.pose.orientation.z
    quat[3] = msg.pose.orientation.w
    
    time_pose = msg.header.stamp   
    

# Callback for velocity
def velCheck(msg):

    global linear_vel
    global ang_vel
    global time_vel
    
    linear_vel[0] = msg.twist.linear.x
    linear_vel[1] = msg.twist.linear.y
    linear_vel[2] = msg.twist.linear.z
    
    ang_vel[0] = msg.twist.angular.x
    ang_vel[1] = msg.twist.angular.y
    ang_vel[2] = msg.twist.angular.z
    
    time_vel = msg.header.stamp
    
    
# Callback for acceleration
def imuCheck(msg):

    global linear_acc
    global time_imu
    
    linear_acc[0] = msg.linear_acceleration.x
    linear_acc[1] = msg.linear_acceleration.y
    linear_acc[2] = msg.linear_acceleration.z
    
    time_imu = msg.header.stamp
    


# -- setPose Conditionals --


def getTargetPose():

    global target_pose, phase
    
    return target_pose[phase]
    

def phaseConversion(target_vel):
    
    global phase, hover_phase, hover_count, target_pose , pos ,height, step_time
    
    if phase == 3:
        print('End State Achieved')
        rospy.sleep(10)
        rospy.signal_shutdown("Ended node because the drone has landed")
        
        
    elif isClose(target_pose[phase][2],height,tol=0.1):
        print ('debug: phase' ,phase)
    	if hover_count == 0:
        	print('Entering Hover Phase.')
        	
        hover_phase = 1
        hover_count += 1
        print('debug: hover: count {}'.format(hover_count))
        if hover_count == 20:
            print('Switching Phase {} to Phase {}'.format(phase,phase+1))
            phase += 1
            hover_count = 0
            hover_phase = 0
                    
    elif phase == 1:
        if step_time == 20*20:
        	phase = 3
        step_time += 1
        print("Simulation done at {} %".format(step_time/400.*100))
        	


# -- setVel Algorithm --



# -- Main --


def main():

    # CSV File Format
    fieldnames = ['phase', '-', '-',
                  'time_sonar','-', 'h_sonar', '-', '-',
                  'time_pose', '-', 'x', 'y', 'z', '-',
                  'quat_x', 'quat_y', 'quat_z', 'quat_w', '-', '-',
                  'time_vel', '-', 'x_dot', 'y_dot', 'z_dot', '-',
                  'ang_x_dot', 'ang_y_dot', 'ang_z_dot', '-','-',
                  'time_imu', 'x_dot2', 'y_dot2', 'z_dot2','-','-',
                  'in_x_dot', 'in_y_dot', 'in_z_dot']
                 
    csvfile = open('logging_state{}{}{}.csv'.format(TARGET[0],TARGET[1],TARGET[2]), 'w')    
    writer = csv.DictWriter(csvfile, fieldnames = fieldnames)
    writer.writeheader()
                 



    global nodeState #state of the node
    global step_time
    step_time = 0
    
    rospy.init_node('navigator',disable_signals=True)   # make ros node
    
	# rate will update publisher at 20hz, higher than the 2hz minimum before tieouts occur
    rate = rospy.Rate(nodeState['rate']) 
    
    stateManagerInstance = stateManager(rate) # create new statemanager


    # Subscriptions
    #get autopilot state including arm state, connection status and mode
    rospy.Subscriber("/mavros/state", State, stateManagerInstance.stateUpdate)  
    #get current distance from ground
    rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, heightCheck)  
    #subscribe to position messages
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
    
    # Instantiate PID Controller with weights
    pid_x = PID_controller(nodeState['rate'],-0.5,-0.3,0,maxVel=0.5)
    pid_y = PID_controller(nodeState['rate'],-0.5,-0.5,0)
    pid_z = PID_controller(nodeState['rate'],-0.5,-0.3,0)
    
    

    # Control Loop
    while not rospy.is_shutdown():      
        
        controller.publishTargetPose(stateManagerInstance)
        stateManagerInstance.incrementLoop()
        rate.sleep()

        if stateManagerInstance.getLoopCount() > 100:   
        	#need to send some position data before we can switch 
        	#to offboard mode otherwise offboard is rejected
            
            global pos, quat, height
            global ang_vel, linear_vel
            global linear_acc
            global time_sonar, time_pose, time_vel, time_imu
            global phase
            
            
            
            print('debug Position: X = {}, Y = {}, Z = {}'.format(pos[0], pos[1], height))
            
            print('Linear Velocity: X = {}, Y = {}, Z = {}'.format(
                   linear_vel[0],linear_vel[1],linear_vel[2])
                   )
            
            print('Quaternions: X = {}, Y = {}, Z = {}, W = {}'.format(
                   quat[0], quat[1], quat[2], quat[3])
                   )
            
            print('Angular Velocity: X = {}, Y = {}, Z = {}'.format(
                   ang_vel[0], ang_vel[1], ang_vel[2])
                   )
            
            print('Acceleration: X = {}, Y = {}, Z = {}'.format(
                   linear_acc[0], linear_acc[1], linear_acc[2])
                   )
            
            
            target_pose = getTargetPose() # target displacement
            print('debug',target_pose)          
            if phase == 1:
           		target_vel = target_pose
            else:
                target_vel = [pid_x(pos[0], target_pose[0]), # get target velocity
                			  pid_y(pos[1], target_pose[1]), 
                			  pid_z(height, target_pose[2])]
            			  
            writerdict = {'h_sonar': height,
                 'x': pos[0], 'y': pos[1], 'z': pos[2],
                 'x_dot': linear_vel[0], 'y_dot': linear_vel[1], 'z_dot': linear_vel[2] ,
                 'x_dot2': linear_acc[0], 'y_dot2': linear_acc[1], 'z_dot2': linear_acc[2],
                 'quat_x': quat[0], 'quat_y': quat[1], 'quat_z': quat[2], 'quat_w': quat[3],
                 'ang_x_dot': ang_vel[0], 'ang_y_dot': ang_vel[1], 'ang_z_dot': ang_vel[2],
                 'time_sonar': time_sonar, 'time_pose': time_pose, 'time_vel': time_vel, 'time_imu': time_imu,
                 'in_x_dot': target_vel[0], 'in_y_dot': target_vel[1], 'in_z_dot': target_vel[2],
                 'phase': phase}
            
            writer.writerow(writerdict)
            			  
            controller.setVel(target_vel) #send input [vx,vy,vz]
            phaseConversion(target_vel)	  #check phase state, may shudown the state
            
            if rospy.is_shutdown():
				break
				print('IN LOOP!!')
				csvfile.close()
    
            
            stateManagerInstance.offboardRequest()  #request control from external computer
            stateManagerInstance.armRequest()   #arming must take place after offboard is requested
    

       
    rospy.spin()    #keeps python from exiting until this node is stopped


if __name__ == '__main__':
    main()
