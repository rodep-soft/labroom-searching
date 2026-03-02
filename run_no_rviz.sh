#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROS_WS_DIR="$SCRIPT_DIR/ros_ws"

if [ ! -f "$ROS_WS_DIR/install/setup.bash" ]; then
  echo "Error: $ROS_WS_DIR/install/setup.bash が見つかりません"
  echo "先に ros_ws で colcon build を実行してください"
  exit 1
fi

source "$ROS_WS_DIR/install/setup.bash"

if ! ros2 pkg prefix bringup >/dev/null 2>&1; then
  echo "bringup package が見つからないためビルドします..."
  mkdir -p "$ROS_WS_DIR/.colcon_log"
  (
    cd "$ROS_WS_DIR"
    COLCON_LOG_PATH="$ROS_WS_DIR/.colcon_log" colcon build --packages-select bringup
  )
  source "$ROS_WS_DIR/install/setup.bash"
fi

# rviz2 を含まない構成を起動（bringup/main.launch.py）
exec ros2 launch bringup main.launch.py
