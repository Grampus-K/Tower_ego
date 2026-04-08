sudo chmod 777 /dev/ttyACM1 & sleep 4;
roslaunch realsense2_camera rs_camera.launch camera:=cam_front serial_no:=147122074245 & sleep 8;
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM1" & sleep 8;
roslaunch ego_planner single_run_in_exp.launch & sleep 1;
roslaunch ego_planner rviz.launch

wait;
