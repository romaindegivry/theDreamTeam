#!/bin/sh
trap "kill 0" SIGINT
echo "Launch Gazebo"
(
cd ~/src/Firmware
./gazeboVmWork.sh "make posix_sitl_default gazebo_iris_opt_flow"
)&
sleep 15s
echo "Launch Drone Node"
(
cd ~/src/Firmware
./rosLaunchDrone.sh
)&
