#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include "input/mobile_io.hpp"

// Helper to apply a "dead zone" to an input value
float applyDeadZone(float raw, float dead_zone) {
  if (std::abs(raw) < dead_zone)
    return 0;
  // Scale from [dead_zone, 1] to [0, 1] (and same for [-1, -dead_zone]
  if (raw > 0)
    return (raw - dead_zone) / (1 - dead_zone);
  return (raw + dead_zone) / (1 - dead_zone);
}

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "controller");
  ros::NodeHandle node;

  // Create mobile IO connection
  auto io = hebi::MobileIO::create("Daisy", "mobileIO");
  // Configure the mobile IO app; keep trying until we succeed for each of these!
  while (!io->setSnap(3, 0)) { ROS_WARN("Error while setting Mobile IO state"); }
  while (!io->setButtonMode(1, hebi::MobileIO::ButtonMode::Toggle)) { ROS_WARN("Error while setting Mobile IO state"); }
  while (!io->setButtonOutput(1, 1)) { ROS_WARN("Error while setting Mobile IO state"); }
  while (!io->setButtonOutput(8, 1)) { ROS_WARN("Error while setting Mobile IO state"); }

  using ButtonState = hebi::MobileIODiff::ButtonState;

  // Initialize ROS interface
  ros::Publisher velocity_pub = node.advertise<geometry_msgs::Twist>("velocity_command", 100);
  geometry_msgs::Twist velocity_msg;

  ros::Publisher mode_select_pub = node.advertise<std_msgs::Bool>("mode_select", 100);
  std_msgs::Bool mode_select_msg;

  /////////////////// Main Loop ///////////////////

  // Scale the joystick scale to motion of the robot in SI units (m/s, rad/s,
  // etc).
  static constexpr float xyz_scale {0.175};
  static constexpr float rot_scale {0.4};

  // Main command loop
  auto last_state = io->getState();
  bool got_feedback{};
  while(ros::ok()) {

    // Get next IO feedback state
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    auto state = io->getState(got_feedback);
    if (!got_feedback) {
      velocity_msg.linear.x = 0;
      velocity_msg.linear.y = 0;
      velocity_msg.linear.z = 0;
   
      velocity_msg.angular.y = 0;
      velocity_msg.angular.z = 0;

      velocity_pub.publish(velocity_msg);
      continue;
    }
    hebi::MobileIODiff diff(last_state, state);
    last_state = state;

    // Send messages according to these diffs...
    velocity_msg.linear.x = state.getAxis(8) * xyz_scale;
    velocity_msg.linear.y = -state.getAxis(7) * xyz_scale;
    velocity_msg.linear.z = applyDeadZone(state.getAxis(3), 0.25) * xyz_scale;
   
    velocity_msg.angular.y = rot_scale * state.getAxis(2),
    velocity_msg.angular.z = -rot_scale * state.getAxis(1);

    velocity_pub.publish(velocity_msg);

    if (diff.getButton(1) == ButtonState::ToOn) {
      mode_select_msg.data = true;
      mode_select_pub.publish(mode_select_msg);
    } else if (diff.getButton(1) == ButtonState::ToOff) {
      mode_select_msg.data = false;
      mode_select_pub.publish(mode_select_msg);
    }

    // Quit
    if (diff.getButton(8) == ButtonState::ToOn) {
      break;
    }

    // Call any pending callbacks (note -- there are none right now)
    ros::spinOnce();
  }

  return 0;
}
