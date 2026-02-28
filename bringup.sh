foot () {
        ./mecanum/robot &
}

mapping () {
        source ./ros_ws/install/setup.bash
        ros2 launch brinup
}

sudo chmod 777 /dev/ttyACM*
fastdds discover --server-id 0 > /tmp/fastdds_server.log 2>&1 &
export ROS_DISCOVERY_SERVER="100.85.201.109:11811"

foot
mapping
