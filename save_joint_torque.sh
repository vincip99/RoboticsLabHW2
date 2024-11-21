#!/bin/bash

# Ensure ROS 2 environment is sourced
source /opt/ros/humble/setup.bash

# Enter number file
read -p "Enter a number to add to the file name: " num

# Record specified topics into a bag file named "joint_torque"
ros2 bag record /effort_controller/commands \
         -o "bag_records/joint_torques_bag_${num}"
