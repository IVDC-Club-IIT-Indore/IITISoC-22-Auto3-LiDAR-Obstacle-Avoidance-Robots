# hebi_gazebo
### NOTE: Intended for use with Gazebo versions 7 (Ubuntu 16.04) and 9 (Ubuntu 18.04) only
This repository holds the plugins and configuration files used to simulate HEBI robots in Gazebo.
In addition, you will need the model files contained in the [hebi_description](https://github.com/HebiRobotics/hebi_description) package, or your own hrdf/urdf in the case of a custom robot.
You may also want to download [hebi_moveit_configs](https://github.com/HebiRobotics/hebi_moveit_configs) if you are interested in Moveit! integration.

These instructions have been tested on Ubuntu 18.04 (Gazebo9/ROS Melodic) and Ubuntu 16.04 (Gazebo7/ROS Kinetic), and while other OS/Gazebo combinations may work they are untested (use at your own risk).

## Usage Guide
While this package is compatible with ROS simulation methods, the HEBI Gazebo plugin is fully independent of ROS, and can be used without it. Both use cases are described below:

### Simulating with ROS
1. Gazebo needs to know the path to the HEBI plugin and models. This is done by setting the GAZEBO_PLUGIN_PATH and GAZEBO_MODEL_PATH environment variables.
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/path/to/your/ros_ws/src/hebi_gazebo/plugin/gazebo9
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/path/to/your/ros_ws/src
```
Note that the plugin path should point to the directory under hebi_gazebo/plugin/ that matches your version of Gazebo. For ROS Melodic, the default is Gazebo9, for ROS Kinetic the default is Gazebo7.

2. Make sure that `hebi_gazebo`, `[hebi_description](https://github.com/HebiRobotics/hebi_description)`, and `[hebi_cpp_api_ros_examples](https://github.com/HebiRobotics/hebi_cpp_api_ros_examples)` are cloned and built in your current, sourced workspace.
3. To test, run the following command:
`roslaunch hebi_gazebo arm_simulation.launch arm_type:=A-2085-06`
4. You should see the 6-DoF arm kit rendered in Gazebo. A `rostopic list` should show several topic-based interfaces for controlling the arm.
Note that the ROS nodes for simulation are identical to those used for controlling HEBI hardware.

### Simulating without ROS
*NOTE: Currently the hebi_description pipeline that generates SDF files is not working properly without a minimal ROS install. Specifically, the xacro conversion process uses rospack to find hebi_description. Once the SDF file is generated, there is no direct dependency on ROS.*

1. Install the gazebo simulator. Follow the instructions at http://gazebosim.org/tutorials?tut=install_ubuntu
2. Clone/download hebi_gazebo. This does not need to be in any particular location. hebi_gazebo/models contains several subdirectories, one for each HEBI kit model.
3. Gazebo needs to be told the location of the HEBI plugin and models. To do this, put the following line in your ~/.bashrc file:
```
source /path/to/hebi_gazebo/scripts/setup_environment.bash
```

4. You can load your model into Gazebo in one of two ways:
    1. run gazebo from the terminal `gazebo`.
The second tab in the left panel menu, "Insert", will have several locations it can find models on your system.
The HEBI SDF models should appear here if your environment variables are set.
    2. Write a .world file that includes the model. For an example of this, see hebi_gazebo/worlds/scara_world.world

5. At this point, your simulated robot can be interacted with through standard HEBI tools such as Scope or the programmatic APIs. The arm will have the family name "Arm".
