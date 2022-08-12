#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "hebi_cpp_api_examples/SaveWaypoint.h"
#include "hebi_cpp_api_examples/Playback.h"

#include "src/util/mobile_io.hpp"

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "controller");
  ros::NodeHandle node;

  std::string family;
  if (node.hasParam("family") && node.getParam("family", family)) {
    ROS_INFO("Found and successfully read 'family' parameter");
  } else {
    ROS_INFO("Could not find/read 'family' parameter; defaulting to 'Rosie'");
    family = "Rosie";
  }

  // Create mobile IO connection
  auto io = hebi::MobileIO::create(family, "mobileIO");
  using ButtonState = hebi::MobileIODiff::ButtonState;

  // TODO: set IO LED green when found, white when exiting?
   // Configure the mobile IO app; keep trying until we succeed for each of these!
   //
  for (int i = 1; i <= 7; ++i)
    while (!io->setButtonMode(i, hebi::MobileIO::ButtonMode::Momentary)) { ROS_WARN("Error while setting Mobile IO state"); }
  // The "teach mode" toggle:
  while (!io->setButtonMode(7, hebi::MobileIO::ButtonMode::Toggle)) { ROS_WARN("Error while setting Mobile IO state"); }

  for (int i = 1; i <= 6; ++i)
    while (!io->setButtonOutput(i, 0)) { ROS_WARN("Error while setting Mobile IO state"); }
  while (!io->setButtonOutput(7, 1)) { ROS_WARN("Error while setting Mobile IO state"); }
  while (!io->setButtonOutput(8, 1)) { ROS_WARN("Error while setting Mobile IO state"); }

  /////////////////// Initialize ROS interface ///////////////////

  // Base
  ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  geometry_msgs::Twist cmd_vel_msg;

  // Gripper
  ros::Publisher gripper_pub = node.advertise<std_msgs::Float64>("gripper_strength", 100);
  std_msgs::Float64 gripper_msg;

  // Arm (teach/repeat)
  ros::Publisher compliant_mode_pub = node.advertise<std_msgs::Bool>("compliant_mode", 100);
  std_msgs::Bool compliant_mode_msg;

  ros::Publisher add_waypoint_pub = node.advertise<hebi_cpp_api_examples::SaveWaypoint>("save_waypoint", 100);
  hebi_cpp_api_examples::SaveWaypoint add_waypoint_msg;

  ros::Publisher playback_pub = node.advertise<hebi_cpp_api_examples::Playback>("playback", 100);
  hebi_cpp_api_examples::Playback playback_msg; 
  // Right now, we only playback waypoint
  playback_msg.mode = hebi_cpp_api_examples::Playback::GO_TO_WAYPOINT;

  /////////////////// Main Loop ///////////////////

  // Main command loop
  auto last_state = io->getState();

  uint8_t send_count = 0;
  uint8_t send_period = 10;

  bool got_feedback{};

  while(ros::ok()) {
    // Get next IO feedback state
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)

    auto state = io->getState(got_feedback);
    hebi::MobileIODiff diff(last_state, state);
    last_state = state;

    // Base
    cmd_vel_msg.linear.y = -state.axes_[6];
    cmd_vel_msg.linear.x = state.axes_[7];
    cmd_vel_msg.angular.z = -state.axes_[0] * 2.0; 
    if (send_count == 0 && got_feedback)
      cmd_vel_pub.publish(cmd_vel_msg);

    // Gripper
    gripper_msg.data = state.axes_[5];
    gripper_pub.publish(gripper_msg);

    // Arm "compliant mode"
    auto b7 = diff.get(7);
    if (b7 == ButtonState::ToOn) {
      compliant_mode_msg.data = true;
      compliant_mode_pub.publish(compliant_mode_msg);
    } else if (b7 == ButtonState::ToOff) {
      compliant_mode_msg.data = false;
      compliant_mode_pub.publish(compliant_mode_msg);
    }

    // Waypoint - buttons B1-B6
    auto b1 = diff.get(1);
    auto b2 = diff.get(2);
    auto b3 = diff.get(3);
    auto b4 = diff.get(4);
    auto b5 = diff.get(5);
    auto b6 = diff.get(6);
    // Save waypoint when button B7 is pressed down, playback
    // when it is up
    bool in_teach_mode = state.buttons_[6];
    if (in_teach_mode) {
      if (b1 == ButtonState::ToOn) {
        add_waypoint_msg.name = "1";
        add_waypoint_pub.publish(add_waypoint_msg);
      }
      if (b2 == ButtonState::ToOn) {
        add_waypoint_msg.name = "2";
        add_waypoint_pub.publish(add_waypoint_msg);
      }
      if (b3 == ButtonState::ToOn) {
        add_waypoint_msg.name = "3";
        add_waypoint_pub.publish(add_waypoint_msg);
      }
      if (b4 == ButtonState::ToOn) {
        add_waypoint_msg.name = "4";
        add_waypoint_pub.publish(add_waypoint_msg);
      }
      if (b5 == ButtonState::ToOn) {
        add_waypoint_msg.name = "5";
        add_waypoint_pub.publish(add_waypoint_msg);
      }
      if (b6 == ButtonState::ToOn) {
        add_waypoint_msg.name = "6";
        add_waypoint_pub.publish(add_waypoint_msg);
      }
      // Blue if in teach mode
      io->setLedColor(0, 0, 255);
    } else {
      if (b1 == ButtonState::ToOn) {
        playback_msg.name = "1";
        playback_pub.publish(playback_msg);
      }
      if (b2 == ButtonState::ToOn) {
        playback_msg.name = "2";
        playback_pub.publish(playback_msg);
      }
      if (b3 == ButtonState::ToOn) {
        playback_msg.name = "3";
        playback_pub.publish(playback_msg);
      }
      if (b4 == ButtonState::ToOn) {
        playback_msg.name = "4";
        playback_pub.publish(playback_msg);
      }
      if (b5 == ButtonState::ToOn) {
        playback_msg.name = "5";
        playback_pub.publish(playback_msg);
      }
      if (b6 == ButtonState::ToOn) {
        playback_msg.name = "6";
        playback_pub.publish(playback_msg);
      }
      // Green if in playback mode
      io->setLedColor(0, 255, 0);
    }

    // Quit
    if (diff.get(8) == ButtonState::ToOn) {
      break;
    }

    ++send_count;
    send_count = send_count % send_period;

    // Call any pending callbacks (note -- there are none right now)
    ros::spinOnce();
  }
      
  io->setLedColor(255, 255, 255);

  return 0;
}
