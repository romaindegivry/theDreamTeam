#This scripts initialises gazebo and logs the state of the simulation
HERE=$(pwd)
echo "Launch Gazebo"
cd ~/src/Firmware
gnome-terminal -e './gazeboVmWork.sh "make posix_sitl_default gazebo_iris_opt_flow"'
sleep 10s

echo "Launch Drone Node"
cd ~/src/Firmware
gnome-terminal -e './rosLaunchDrone.sh'
sleep 10s

echo 'Initializing the logs'
NOW=$(date +"%Y_%m_%d__%H_%M_%S")
gz log -d 1 -o ./logs/$NOW.log
sudo trap 'echo "SIGINT recived, ending the logging"; gz log -d 0'

echo "Launch Navigator"
cd $HERE/baselinescripts
gnome-terminal -e 'python navigator.py'
sleep 300s

echo 'stop logging after 5 mins'
gz log -d 0

