# User Overview: Rosie Teach/Repeat

This example demonstrate control of a Rosie HEBI Kit (a 6-DoF arm + gripper on top of an omnidirectional base).

## To run:

First navigate to your existing catkin workspace (e.g., `/home/hebi/rosie_ws` for a HEBI preinstalled Rosie kit), source your workspace setup file (e.g., `source devel/setup.sh`), and then run the following to start:

```
roslaunch hebi_cpp_api_examples rosie_teach_repeat.launch
```

If the system does not start, check the console for error messages; it is most likely that a module was not found on the network.

Once connected, the state of several of the Mobile IO controls will be configured, and the border should turn green.

## To command:

You must have a device running the free HEBI Mobile IO Android/iOS app.  This should be connected to Rosie's WiFi network, and be configured to have family "Rosie" and name "mobileIO".

The controls are are follows:

* `A1`: rotate the base left/right
* `A6`: open and close the gripper (up closes, down opens)
* `A7`/`A8`: translate the base in x/y
* `B1`-`B6`: in _playback mode_, attempts to move to the numbered waypoint, in _training mode_, saves the current position as a numbered waypoint.
* `B7`: toggle the active mode between "playback" (default; border is green) and "teaching" (border is blue).  In "teaching" mode, the arm is compliant and roughly weightless, so can easily be moved around.
* `B8`: quit

# Developer Overview: Rosie Teach/Repeat

## Topics (inherited from the omni base and teach repeat nodes)

### `/base/cmd_vel` -- `geometry_msgs::Twist`

This is the commanded velocity for the omnibase.  Note that this must be continually sent at 1-10 Hz, or the system will deccelerate to a velocity of zero for safety.

This channel is appropriate for use with the ROS nav stack.

### `/base/motion` -- action `hebi_cpp_api_examples::BaseMotion`

This commands discrete (x,y,theta) motions, relative to the current pose of the robot.

### `/arm/compliant_mode` -- `std_msgs::Bool`

Set this to `true` to make the arm go into a "gravity compensated compliant mode".  It can be easily moved around by hand in this mode (but may drift slightly).  Set this to `false` to exit this mode.

### `/arm/save_waypoint` -- `hebi_cpp_api_examples::SaveWaypoint`

Save the current position of the arm as a waypoint with the given name.  This writes the waypoint to disk.  In this demo, the waypoint names match the number of the button pressed to save them.

### `/arm/playback` -- `hebi_cpp_api_examples::Playback`

Command the arm to move to the waypoint with the given name.  If no such waypoint exists, move to the first saved waypoint.  If no waypoints exist, do nothing.

### `/gripper/gripper_strength` -- `std_msgs::Float64`

Command the strength of the gripper jaws, from `-1` (all the way open) to `1` (closing with the maximum configured force).

## Nodes:

### */base/omni_base_node* -- `hebi_cpp_api_examples::omni_base_node`

Moves the base around using actions or messages.  Can optionally publish odometry data if the "publish_odom" parameter is set to tru.

### */arm/teach_repeat_node* -- `hebi_cpp_api_examples::teach_repeat_node`

Controls the arm, saving waypoints and paths for later playback.

### */gripper/gripper_node* -- `hebi_cpp_api_examples::gripper_node`

Opens and closes the gripper with a specified amount of effort.

### */rosie_teach_repeat_controller_node* -- `hebi_cpp_api_examples::rosie_teach_repeat_controller_node`

Node that translates input from the Mobile IO app to ROS topics.

## To customize:

Any of the nodes that are running can be run individually, and you can send messages on the above ROS topics from your own nodes, as well.

Finally, you can edit the individual notes themselves to customize the behavior you see, or use them as a starting point to write your own.

Note that most of the nodes have parameters that allow you to customize the modules, gain and config files, starting waypoints, etc at launch time.

