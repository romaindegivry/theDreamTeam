#!/usr/bin/env python

import subprocess as sub
import os
import rospy
import math
import sys
import time

steps = [[1,0,0],[0,1,0],[0,0,1]]

for step in steps:
	#start gazebo/ros service
	proc = sub.Popen('bash ../../bash_scripts/run_gazebo.sh',shell=True)
	print('Done')
	time.sleep(1) #wait a second for gazebo to load

	#when target  altitude reached (ie 1m) hold position

	#generate input
	#log time

	#takeoff
	os.system('python control_logger.py {}'.format(step))

	#kill all gazebo/ros services
	n=25
	while n>0:
		n -= 1 
		print('Kill in {}s'.format(n))
		time.sleep(1)
	rospy.signal_shutdown("Killed control.py node because process ended")
	proc.terminate()
	proc.kill()
	sub.call('rosnode kill --all',shell=True)
	sub.call('killall -9 px4',shell=True)

	time.sleep(2) #wait a sec before the end
	sub.call('clear',shell=True)

	print('termination done')
	#takeoff
	time.sleep(2) #wait a sec before the end
