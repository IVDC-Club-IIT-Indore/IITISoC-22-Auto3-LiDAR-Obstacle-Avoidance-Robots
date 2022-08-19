# Overview

These examples demonstrate use of the C++ API directly from ROS nodes. These wrap up this functionality and low-level module communication, and expose a higher-level "system" interface on ROS topics, actions, and services.

**Important: to run any of these commands, you will need to run `source devel/setup.sh` from your catkin workspace first**

Kit examples are provided for [arms](kits/arm/README.md) and [mobile bases](kits/base/README.md).

For more information, go to http://wiki.ros.org/hebi_cpp_api_examples.

## Organization

* `kits`: source code for the nodes that control HEBI kits.
* `util`: helper functions or classes for interfacing with HEBI hardware or ROS topics.  Some of these functions/classes will move into the HEBI C++ API in future revisions.

