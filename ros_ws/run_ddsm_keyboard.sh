#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
  echo "Error: $SCRIPT_DIR/install/setup.bash が見つかりません"
  echo "先に ros_ws で colcon build を実行してください"
  exit 1
fi

if ! command -v fastdds >/dev/null 2>&1; then
  echo "Error: fastdds コマンドが見つかりません"
  exit 1
fi

source "$SCRIPT_DIR/install/setup.bash"

# Fast DDS Discovery Server 起動
fastdds discovery --server-id 0 &
FASTDDS_PID=$!

# DDSM launch 起動
ros2 launch "$SCRIPT_DIR/launch/ddsm_standalone.launch.py" &
DDSM_PID=$!

cleanup() {
  kill "$DDSM_PID" 2>/dev/null || true
  kill "$FASTDDS_PID" 2>/dev/null || true
}
trap cleanup EXIT

# Keyboard launch は前面実行
exec ros2 launch "$SCRIPT_DIR/launch/keyboard_standalone.launch.py"
