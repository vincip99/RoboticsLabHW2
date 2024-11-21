## :package: Homework2_rl2024
Homework 2 for Robotics Lab 2024/2025

## :hammer: Build
First build all the packages by using:

```
colcon build --packages-select orocos_kdl iiwa_description external_torque_sensor_broadcaster impedance_controller iiwa_bringup ros2_kdl_package python_orocos_kdl 
```
In each terminal you open, source the install directory:
```
source install/setup.bash
```

## :white_check_mark: Start Gazebo and spawn the robot using the effort_controller
To launch `iiwa.launch.py` using as command interface "effort" and as controller "effort_controller", run:
```
ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="effort" robot_controller:="effort_controller"
```
The arguments used are:
 use_sim - Start robot in Gazebo simulation.
 command_interface (default: "position") - Robot command interface [position|velocity|effort].
 robot_controller (default: "iiwa_arm_controller") - Robot controller to start.
 NOTES:
 After running the command provided above, as soon as Gazebo opens, PRESS THE PLAY BUTTON in the lower left corner.This will ensure proper loading and valid activation of the controllers!!!


 
In another terminal run the node, using the following command:
```
ros2 run ros2_kdl_package ros2_kdl_node
```
By default the node publishes joint position commands and sets a trapezoidal-linear trajectory for the robot.

# :white_check_mark: Run the node using the joint space inverse dynamics controller

The joint space inverse dynamics controller is selected by setting cmd_interface:="effort" when running ros2_kdl_node, as follows.

To set a specific trajecory for the joint space controller, run:

CUBIC-CIRCULAR:
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p s_type:="cubic" -p traj_type:="circular" -p cmd_type:="jnt_id" -p cmd_interface:="effort"
```
TRAPEZOIDAL-CIRCULAR:
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p s_type:="trapezoidal" -p traj_type:="circular" -p cmd_type:="jnt_id" -p cmd_interface:="effort"
```

CUBIC-LINEAR:
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p s_type:="cubic" -p traj_type:="linear" -p cmd_type:="jnt_id" -p cmd_interface:="effort"
```

TRAPEZOIDAL-LINEAR:
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p s_type:="trapezoidal" -p traj_type:="linear" -p cmd_type:="jnt_id" -p cmd_interface:="effort"
```

# :white_check_mark: Run the node using the cartesian space inverse dynamics controller

The joint space inverse dynamics controller is selected by setting cmd_interface:="effort" when running ros2_kdl_node, as follows.

To set a specific trajecory for the cartesian space controller, run:

CUBIC-CIRCULAR:
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p s_type:="cubic" -p traj_type:="circular" -p cmd_type:="op_id" -p cmd_interface:="effort"
```
TRAPEZOIDAL-CIRCULAR:
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p s_type:="trapezoidal" -p traj_type:="circular" -p cmd_type:="op_id" -p cmd_interface:="effort"
```

CUBIC-LINEAR:
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p s_type:="cubic" -p traj_type:="linear" -p cmd_type:="op_id" -p cmd_interface:="effort"
```

TRAPEZOIDAL-LINEAR:
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p s_type:="trapezoidal" -p traj_type:="linear" -p cmd_type:="op_id" -p cmd_interface:="effort"
```

