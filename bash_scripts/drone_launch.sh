#!/bin/sh
echo "Launch Gazebo"
cd ~/src/Firmware
gnome-terminal -e './gazeboVmWork.sh "make posix_sitl_default gazebo_iris_opt_flow"'
sleep 10s
echo "Launch Drone Node"
cd ~/src/Firmware
gnome-terminal -e './rosLaunchDrone.sh'
sleep 10s
echo "Launch Navigator"
cd ~/catkin_ws/src/l3drone/scripts/theDreamTeam/baselineScripts
gnome-terminal -e 'python navigator_pid.py |cat > bash_scripts.txt | tee navigator_log.txt & disown'
