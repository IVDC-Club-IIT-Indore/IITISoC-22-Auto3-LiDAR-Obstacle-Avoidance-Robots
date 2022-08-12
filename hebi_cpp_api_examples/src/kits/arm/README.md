# Overview

These examples demonstrate use of the C++ API directly from ROS nodes. These wrap up this functionality and low-level module communication, and expose a higher-level "system" interface on ROS topics, actions, and services.

**Important: to run any of these commands, you will need to run `source devel/setup.sh` from your catkin workspace first**

# Arm (`arm_node`)

See http://wiki.ros.org/hebi_cpp_api_examples/ArmNode

# MoveIt Arm Node (`moveit_arm_node`)

## Requirements:

Set up as with the `arm_node` above.

In addition, you should have:
- a MoveIt installation
- the `hebi_description` package either installed (sudo apt install `ros-<distro>-hebi_description`) or in your workspace ( https://github.com/HebiRobotics/hebi_description/ )
- the appropriate HEBI moveit config (from the `hebi_moveit_configs` package, https://github.com/HebiRobotics/hebi_moveit_configs/ )

This example can then be controlled with the ROS move group interface.

For more information, see the [HEBI MoveIt Configs Documentation](https://github.com/HebiRobotics/hebi_moveit_configs/README.md).

## To run:

```
roslaunch hebi_cpp_api_examples moveit_arm_node.launch arm_type:=<arm_type> gripper_type:=<gripper_type>
```

Where `arm_type` is one of the HEBI Arm kit types with a matching MoveIt configs (e.g., `a-2085-04`, `a-2085-05`, `a-2085-06`, etc), and `gripper_type` is optional.  Defaults to no gripper; if "parallel" is given, then this pulls the appropriate moveit config.

## To command:

### `hebi_arm_controller/follow_joint_trajectory` action

- This responds to the standard `FollowJointTrajectory` action used by MoveIt.

## Feedback:

### `joint_states`

- This publishes position, velocity, and effort on the `/joint_states` channel.  You may need a robot state publisher running to transform these appropriately using `tf` for use in MoveIt.
