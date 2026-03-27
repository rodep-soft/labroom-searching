# labroom-searching

Building
cd ~/ros_ws/
make
source install/setup.bash

Excuting
ros2 launch bringup main.launch.py
ros2 launch bringup rviz2.launch.py

if you use school wifi, do following commands
server side (robot)
fastdds discovery --server-id 0
control + Z
bg
export ROS_DISCOVERY_SERVER=[here is server ip address]:11811

client side 
export ROS_DISCOVERY_SERVER=[here is server ip address]:11811

https://qiita.com/natzv_jp/items/b89253ba3504cdd2af6b
refer to this page
