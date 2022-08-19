# hebi_cpp_api_ros_examples

Examples of using HEBI components through ROS, using the HEBI C++ API.

This repository demonstrates integration of the HEBI C++ API with ROS, as well as several system-level demonstrations.

**This is intended to be a replacement for the deprecated `hebi_ros` package.**

## Overview

This package provides a collection of nodes which can be used as standalone components of a larger system, or you can use as a starting point for creating your own custom nodes controlling HEBI hardware at a low level.

For full documentation, see the package documentation on the ROS wiki:

http://wiki.ros.org/hebi_cpp_api_examples

If you want to jump straight into the code, check out the `src` folder.

## Automatically starting a HEBI demo configuration with systemd

For kits with included computers, it can be convenient to autostart a demo configuration as soon as the onboard system has booted. To do this, we make use of systemd services. Our structure is based on the one provided by Rover Robotics (https://blog.roverrobotics.com/how-to-run-ros-on-startup-bootup/)

### Configuring autostart
The autostart config relies on 2 services: `roscore.service` and `roslaunch.service` that we will create.

To create `roscore.service`, create a file at `/etc/systemd/system/roscore.service` with the following contents:

```
[Unit]
After=NetworkManager.service time-sync.target
[Service]
Type=forking
User=hebi
# Start roscore as a fork and then wait for the tcp port to be opened
# —————————————————————-
# Source all the environment variables, start roscore in a fork
# Since the service type is forking, systemd doesn’t mark it as
# ‘started’ until the original process exits, so we have the
# non-forked shell wait until it can connect to the tcp opened by
# roscore, and then exit, preventing conflicts with dependant services
ExecStart=/bin/sh -c ". /opt/ros/kinetic/setup.sh; . /etc/ros/env.sh; roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done"
[Install]
WantedBy=multi-user.target
```

This service references `/etc/ros/env.sh`, which defines the environment variables `ROS_IP` and `ROS_MASTER_URI`. These are needed when running multi-computer ROS configurations. For HEBI kits, `env.sh` should look like
```
#!/bin/sh
export ROS_IP=10.10.1.2
export ROS_MASTER_URI=http://10.10.1.2:11311
```

after these files are created, run `sudo systemctl enable roscore.service` to enable the roscore service.

Next create `/etc/systemd/system/roslaunch.service` containing

```
[Unit]
Requires=roscore.service
PartOf=roscore.service
After=NetworkManager.service time-sync.target roscore.service
[Service]
Type=simple
User=hebi
ExecStart=/usr/sbin/roslaunch
[Install]
WantedBy=multi-user.target
```

note the reference to `/usr/sbin/roslaunch`. Now create that file:

```
#!/bin/bash
source ~/workspaces/melodic_ws/devel/setup.bash
source /etc/ros/env.sh
export ROS_HOME=$(echo ~hebi)/.ros
roslaunch <YOUR_LAUNCHFILE_HERE> &
PID=$!
wait "$PID"
```

The second line should be updated to match the location of your ROS workspace, and the roslaunch line with whatever launchfile you would like to run on system boot.

Make sure this file is executable by running `sudo chmod +x /usr/sbin/roslaunch` 

Finally, run `sudo systemctl enable roslaunch.service` to enable the roslaunch service.

Next time you boot the onboard PC, the two services created should start a `roscore` process, and `roslaunch` the file you indicated.

### Disabling/Enabling Autostart

If your kit has been configured to autostart a demo, and you would like to disable this behavior, run the following commands:

`sudo systemctl disable roscore.service`

`sudo systemctl disable roslaunch.service`

The launchfile and roscore will no longer start automatically on boot. To reenable the original behavior, run 

`sudo systemctl enable roscore.service`

`sudo systemctl enable roslaunch.service`
