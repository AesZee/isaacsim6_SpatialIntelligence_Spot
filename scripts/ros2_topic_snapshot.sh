#!/usr/bin/env bash
set -u

# Print a read-only ROS2 topic snapshot for Isaac Sim sensor validation.
# This script does not install packages, does not require sudo, and does not
# record rosbag files. Run it after sourcing ROS2 and starting Isaac Sim's ROS2
# bridge.

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
echo "== ros2 version =="
ros2 --version || true

echo
echo "== Topics with types =="
ros2 topic list -t

echo
echo "== Common sensor topic hints =="
ros2 topic list -t | grep -Ei 'point|cloud|scan|lidar|os1|image|camera|depth|imu|clock|tf' || true

echo
echo "== Next manual checks =="
echo "ros2 topic type <topic>"
echo "ros2 topic hz <topic>"
echo "ros2 topic echo <topic> --field header --once"
