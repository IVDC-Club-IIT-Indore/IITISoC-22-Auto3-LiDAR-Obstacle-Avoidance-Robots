^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package interactive_marker_twist_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.2 (2021-02-03)
------------------
* Remove the leading '/' from the default link; this causes errors in Noetic
* Bump CMake version to avoid CMP0048 warning.
* Contributors: Chris Iverach-Brereton, Tony Baltovski

1.2.1 (2021-01-26)
------------------
* Added tf as a dep.
* Contributors: Tony Baltovski

1.2.0 (2017-06-22)
------------------
* Fixed CMake warning and updated to package format 2. (`#11 <https://github.com/ros-visualization/interactive_marker_twist_server/issues/11>`_)
* Install the default config files. (`#10 <https://github.com/ros-visualization/interactive_marker_twist_server/issues/10>`_)
* Contributors: Tony Baltovski

1.1.0 (2016-12-20)
------------------
* Added additional degrees of freedom (YZ) to linear interactive markers. (`#7 <https://github.com/ros-visualization/interactive_marker_twist_server/issues/7>`_)
* Fix YZ rotation. (`#6 <https://github.com/ros-visualization/interactive_marker_twist_server/issues/6>`_)
* Contributors: Mike Purvis, Paul Bovbel, Tony Baltovski

1.0.0 (2014-03-31)
------------------
* Add roslaunch file check, roslint
* Make cmd_vel topic relative; fixes `#3 <https://github.com/ros-visualization/interactive_marker_twist_server/issues/3>`_.
* Max speed control, fixes `#1 <https://github.com/ros-visualization/interactive_marker_twist_server/issues/1>`_
* Update copyright.
* Contributors: Mike Purvis

0.0.1 (2013-11-27)
------------------
* Parameterize robot's name and the size of the resulting markers.
* Remove non-markers files from turtlebot_viz, Turtlebot-specific references.
* Starting point is https://github.com/turtlebot/turtlebot_viz
