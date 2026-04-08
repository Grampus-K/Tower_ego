sudo chmod 777 /dev/ttyACM0 & sleep 4;
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0" & sleep 5;
rostopic echo /mavros/global_position/global 
wait;
