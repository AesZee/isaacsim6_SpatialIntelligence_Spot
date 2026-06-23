#!/usr/bin/env bash
set -u

# Print a read-only ROS2 TF snapshot for Isaac Sim sensor validation.
# This script does not install packages, does not require sudo, and does not
# create rosbag recordings. It may create the normal tf2_tools view_frames
# output only if tf2_tools is already installed and the user opts to run the
# command shown at the end.

echo "== ROS2 environment =="
if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ROS_DISTRO is not set. Source ROS2 first, for example:"
  echo '  source /opt/ros/$ROS_DISTRO/setup.bash'
else
  echo "ROS_DISTRO=${ROS_DISTRO}"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source your ROS2 setup file before running this script."
  exit 1
fi

echo
echo "== TF topic presence =="
ros2 topic list -t | grep -E '^/tf|^/tf_static' || true

echo
echo "== /tf type =="
ros2 topic type /tf 2>/dev/null || echo "/tf not available"

echo
echo "== /tf_static type =="
ros2 topic type /tf_static 2>/dev/null || echo "/tf_static not available"

echo
echo "== One /tf sample =="
timeout 5s ros2 topic echo /tf --once 2>/dev/null || echo "No /tf sample received within 5 seconds."

echo
echo "== One /tf_static sample =="
timeout 5s ros2 topic echo /tf_static --once 2>/dev/null || echo "No /tf_static sample received within 5 seconds."

echo
echo "== Optional manual TF tree command =="
echo "ros2 run tf2_tools view_frames"
