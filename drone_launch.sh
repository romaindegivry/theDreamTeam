echo "Launch Gazebo"
cd ~/src/Firmware
gnome-terminal -e './gazeboVmWork.sh "make posix_sitl_default gazebo_iris_opt_flow"'
sleep 10s
echo "Launch Drone Node"
cd ~/src/Firmware
gnome-terminal -e './rosLaunchDrone.sh'
sleep 10s
echo "Launch Navigator"
cd ~/catkin_ws/src/l3drone/scripts
gnome-terminal -e 'python navigator.py'
sleep 2s
echo "Track Sensors"
gnome-terminal -e 'rostopic echo /mavros/imu/data_raw'
gnome-terminal -e 'rostopic echo /mavros/px4flow/raw/optical_flow_rad'
