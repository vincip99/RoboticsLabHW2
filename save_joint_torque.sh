#!/bin/bash

# Ensure ROS 2 environment is sourced
source /opt/ros/humble/setup.bash

# Record specified topics into a bag file named "joint_torque"
ros2 bag record /effort_controller/commands \
         -o joint_torques_bag
