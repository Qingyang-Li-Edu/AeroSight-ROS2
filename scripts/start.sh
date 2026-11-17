#!/bin/bash
# Start robot_state_publisher + TF + RViz2 for the G1.
# Usage: start.sh [--gui]   (--gui = joint_state_publisher_gui sliders instead of the zero-pose publisher)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash

# Render the URDF template with this checkout's mesh directory
URDF=/tmp/g1_29dof.urdf
sed "s#@MESH_DIR@#$ROOT/description/meshes#g" "$ROOT/description/g1_29dof.urdf.in" > "$URDF"

trap 'kill $(jobs -p) 2>/dev/null' EXIT INT TERM

ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat "$URDF")" &
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id world &
ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id pelvis --z 0.75 &

# Something must publish /joint_states; the demos replace this (run_demo.sh stops it first).
if [ "$1" = "--gui" ]; then
  ros2 run joint_state_publisher_gui joint_state_publisher_gui &
else
  ros2 run joint_state_publisher joint_state_publisher &
fi

rviz2 -d "$ROOT/config/g1_robot.rviz"
