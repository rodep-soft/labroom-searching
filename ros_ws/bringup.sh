#!/bin/bash

cd ~/labroom-searching/ros_ws/

foot () {
        #./../original/mecanum/robot &
	ros2 run ddsm_controller ddsm_controller_node &
}

mapping () {
        ros2 launch bringup main.launch.py
}


fastdds discovery --server-id 0 &
sleep 2
export ROS_DISCOVERY_SERVER="100.126.180.124:11811"

source ./install/setup.bash

foot
mapping
