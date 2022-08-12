cmake_minimum_required(VERSION 3.1.3)
project(hebi_cpp_api_examples)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  genmsg
  geometry_msgs
  hebi_cpp_api
  message_generation
  nav_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  ArmMotion.action
  BaseMotion.action
)

add_message_files(
  FILES
  EndPath.msg
  FlipperVelocityCommand.msg
  OffsetPlayback.msg
  Playback.msg
  SaveWaypoint.msg
  StartPath.msg
  TargetWaypoints.msg
  TreadedBaseState.msg
)

add_service_files(
  FILES
  SetCommandLifetime.srv
  SetFeedbackFrequency.srv
  SetGains.srv
  SetIKSeed.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  control_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  hebi_cpp_api
  message_runtime
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  .
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Different nodes for the arm kits
add_executable(mecanum_base_node src/kits/base/mecanum_base_node.cpp src/util/mecanum_base.cpp src/util/odom_publisher.cpp)
add_dependencies(mecanum_base_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(mecanum_base_node ${catkin_LIBRARIES} )

add_executable(omni_base_node src/kits/base/omni_base_node.cpp src/util/omni_base.cpp src/util/odom_publisher.cpp)
add_dependencies(omni_base_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(omni_base_node ${catkin_LIBRARIES} )

add_executable(omni_base_controller_node src/kits/base/omni_base_controller_node.cpp src/util/mobile_io.cpp)
add_dependencies(omni_base_controller_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(omni_base_controller_node ${catkin_LIBRARIES} )

add_executable(rosie_teach_repeat_controller_node src/kits/rosie/teach_repeat_controller_node.cpp src/util/mobile_io.cpp)
add_dependencies(rosie_teach_repeat_controller_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosie_teach_repeat_controller_node ${catkin_LIBRARIES} )

add_executable(diff_drive_node src/kits/base/diff_drive_node.cpp src/util/diff_drive.cpp)
add_dependencies(diff_drive_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(diff_drive_node ${catkin_LIBRARIES} )

add_executable(arm_teach_repeat_node src/kits/arm/arm_teach_repeat_node.cpp src/util/path.cpp src/util/waypoint.cpp)
add_dependencies(arm_teach_repeat_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_teach_repeat_node ${catkin_LIBRARIES} )

add_executable(group_node src/basic/group_node.cpp)
add_dependencies(group_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(group_node ${catkin_LIBRARIES} )

add_executable(arm_node src/kits/arm/arm_node.cpp)
add_dependencies(arm_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_node ${catkin_LIBRARIES} )

add_executable(arm_moveit_node src/kits/arm/arm_moveit_node.cpp)
add_dependencies(arm_moveit_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_moveit_node ${catkin_LIBRARIES} )

add_executable(gripper_node src/kits/arm/gripper_node.cpp)
add_dependencies(gripper_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_node ${catkin_LIBRARIES} )

add_executable(gripper_moveit_node src/kits/arm/gripper_moveit_node.cpp)
add_dependencies(gripper_moveit_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_moveit_node ${catkin_LIBRARIES} )

add_executable(hexapod_control src/kits/daisy/hexapod_control.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kits/daisy/robot/leg.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kits/daisy/robot/hexapod.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kits/daisy/robot/step.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kits/daisy/robot/hexapod_parameters.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kits/daisy/xml_util/pugixml.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kits/daisy/xml_util/xml_helpers.cpp
)
target_include_directories(hexapod_control PRIVATE src/kits/daisy)
add_dependencies(hexapod_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(hexapod_control ${catkin_LIBRARIES})

add_executable(hexapod_controller src/kits/daisy/controller_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kits/daisy/input/mobile_io.cpp
)
target_include_directories(hexapod_controller PRIVATE src/kits/daisy)
add_dependencies(hexapod_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(hexapod_controller ${catkin_LIBRARIES})

add_executable(treaded_base_node
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kits/tready/treaded_base_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kits/tready/robot/treaded_base.cpp)

target_include_directories(treaded_base_node PRIVATE src/kits/tready)
add_dependencies(treaded_base_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(treaded_base_node ${catkin_LIBRARIES})

#add_executable(tready_controller src/kits/tready/controller_node.cpp
#  src/kits/tready/control/tready_arm_joystick_control.cpp
#  src/kits/tready/control/tready_control.cpp
#)
#target_include_directories(tready_controller PRIVATE src/kits/tready)
#add_dependencies(tready_controller ${catkin_EXPORTED_TARGETS})
#target_link_libraries(tready_controller ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
  src/integrations/clearpath_jackal/jackal_arm.launch
  src/integrations/clearpath_jackal/jackal_base_station.launch
  src/integrations/clearpath_jackal/jackal_to_hebi.urdf
  src/integrations/clearpath_jackal/rviz_config.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/integrations
)

install(PROGRAMS src/integrations/clearpath_jackal/jackal_arm_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/daisy.launch
  data/hexapod/gains18.xml
  data/hexapod/gains3.xml
  data/hexapod/hex_config.xml
  data/hexapod/left.hrdf
  data/hexapod/right.hrdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## I don't think this is the _right_ way to do this, but it works...
file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/hexapod/gains18.xml
      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/hexapod/gains3.xml
      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/hexapod/hex_config.xml
      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/hexapod/left.hrdf
      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/hexapod/right.hrdf
      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})


install(TARGETS
  arm_moveit_node
  arm_node
  arm_teach_repeat_node
  diff_drive_node
  group_node
  gripper_moveit_node
  gripper_node
  mecanum_base_node
  omni_base_controller_node
  omni_base_node
  rosie_teach_repeat_controller_node
  hexapod_control
  hexapod_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
