#!/bin/bash

cd /home/user/labroom-searching/ros_ws/

foot () {
        ./../original/mecanum/robot &
}

mapping () {
        source ./install/setup.bash
        ros2 launch bringup main.launch.py
}


fastdds discover --server-id 0 &
sleep 2
export ROS_DISCOVERY_SERVER="100.126.180.124:11811"

foot
mapping
