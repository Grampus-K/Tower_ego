sudo chmod 777 /dev/ttyACM0 & sleep 4;
roslaunch realsense2_camera rs_camera.launch camera:=cam_front serial_no:=147122074245 & sleep 8;
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0" & sleep 8;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
roslaunch vins fast_drone_250.launch & sleep 4;
rosrun vins_to_mavros vins_to_mavros_node & sleep 4;
roslaunch vins rviz.launch & sleep 2;
 

wait;
