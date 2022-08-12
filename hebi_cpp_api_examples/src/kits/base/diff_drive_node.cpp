#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <hebi_cpp_api_examples/BaseMotionAction.h>

#include "actionlib/server/simple_action_server.h"

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/trajectory.hpp"

#include "src/util/diff_drive.hpp"

#include <ros/console.h>
#include <ros/package.h>

namespace hebi {
namespace ros {

class BaseNode {
public:
  BaseNode(DiffDrive& base) : base_(base) {
    Color c;
    base_.clearColor();
  }

  // Returns "false" if we have error...
  bool publishActionProgress(float start_prog, float end_prog)
  {
    hebi_cpp_api_examples::BaseMotionFeedback feedback;
    ::ros::Rate r(10);

    // Wait until the action is complete, sending status/feedback along the way.
    while (true) {
      if (action_server_->isPreemptRequested() || !::ros::ok()) {
        ROS_INFO("Preempted base motion");
        action_server_->setPreempted();
        return false;
        break;
      }
      auto t = ::ros::Time::now().toSec();

      // Publish progress:
      feedback.percent_complete = start_prog + (end_prog - start_prog) * base_.trajectoryPercentComplete(t) / 2.0;
      action_server_->publishFeedback(feedback);
 
      if (base_.isTrajectoryComplete(t)) {
        break;
      }

      // Limit feedback rate
      r.sleep(); 
    }
    return true;

  }

  void startBaseMotion(const hebi_cpp_api_examples::BaseMotionGoalConstPtr& goal) {

    ROS_INFO("Executing base motion action");
    // Note: this is implemented right now as rotation, _THEN_ translation, _THEN_ rotation...

    // Set color:
    Color color;
    if (goal->set_color)
      color = Color(goal->r, goal->g, goal->b, 255);
    base_.setColor(color);

    // Face towards (x, y), assuming we start pointed with the x axis forward and the y axis left:
    bool success = true;
    if (goal->x != 0 || goal->y != 0)
    {
      base_.startRotateBy(std::atan2(goal->y, goal->x),
          ::ros::Time::now().toSec());
      success = publishActionProgress(0.0, 0.33);
      if (success) {
        base_.startMoveForward(std::sqrt(goal->x * goal->x + goal->y * goal->y),
          ::ros::Time::now().toSec());
        success = publishActionProgress(0.33, 0.67);
      }
      if (success) {
        base_.startRotateBy(-std::atan2(goal->y, goal->x) + goal->theta,
          ::ros::Time::now().toSec());
        success = publishActionProgress(0.67, 1.0);
      }
    } else {
      // Pure rotation:
      base_.startRotateBy(goal->theta,
          ::ros::Time::now().toSec());
      success = publishActionProgress(0.0, 1.0);
    }

    base_.clearColor();

    // publish when the base is done with a motion
    ROS_INFO("Completed base motion action");

    hebi_cpp_api_examples::BaseMotionResult result;

    result.success = success;
    action_server_->setSucceeded(result); // TODO: set failed?
  }

  // Set the velocity, canceling any active action
  void updateVelocity(geometry_msgs::Twist cmd_vel) {
    // Cancel any active action:
    if (action_server_->isActive()) {
      ROS_WARN("Switching to twist control, Aborting Active Trajectory");
      action_server_->setAborted();
    }

    // Replan given the current command
    base_.startVelControl(cmd_vel.linear.x, cmd_vel.angular.z, ::ros::Time::now().toSec());
  }

  void setActionServer(actionlib::SimpleActionServer<hebi_cpp_api_examples::BaseMotionAction>* action_server) {
    action_server_ = action_server;
  }

private:
  DiffDrive& base_; 

  actionlib::SimpleActionServer<hebi_cpp_api_examples::BaseMotionAction>* action_server_ {nullptr};
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "diff_drive_node");
  ros::NodeHandle node;

  // Get parameters for name/family of modules; default to standard values:
  std::vector<std::string> families;
  if (node.hasParam("families") && node.getParam("families", families)) {
    ROS_INFO("Found and successfully read 'families' parameter");
  } else {
    ROS_INFO("Could not find/read 'families' parameter; defaulting to 'DiffDrive'");
    families = {"DiffDrive"};
  }

  std::vector<std::string> names;
  if (node.hasParam("names") && node.getParam("names", names)) {
    ROS_INFO("Found and successfully read 'names' parameter");
  } else {
    ROS_INFO("Could not find/read 'names' parameter; defaulting to 'W1_left' and 'W2_right'");
    names = {"W1_left", "W2_right"};
  }

  /////////////////// Initialize base ///////////////////

  // Create base and plan initial trajectory
  std::string error_out;
  auto base = hebi::DiffDrive::create(
    families, // Famil(ies)
    names, // Names
    ::ros::package::getPath("hebi_cpp_api_examples") + "/config/gains/diff_drive_gains.xml", // Gains file
    ros::Time::now().toSec(), // Starting time (for trajectory)
    error_out);
  if (!base) {
    ROS_ERROR_STREAM(error_out);
    ROS_ERROR("Could not initialize base! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }

  /////////////////// Initialize ROS interface ///////////////////
   
  hebi::ros::BaseNode base_node(*base);

  // Action server for base motions
  actionlib::SimpleActionServer<hebi_cpp_api_examples::BaseMotionAction> base_motion_action(
    node, "motion",
    boost::bind(&hebi::ros::BaseNode::startBaseMotion, &base_node, _1), false);

  base_node.setActionServer(&base_motion_action);

  base_motion_action.start();

  // Explicitly set the target velocity
  ros::Subscriber set_velocity_subscriber =
    node.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &hebi::ros::BaseNode::updateVelocity, &base_node);

  /////////////////// Main Loop ///////////////////

  // Main command loop
  while (ros::ok()) {

    auto t = ros::Time::now().toSec();

    // Update feedback, and command the base to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!base->update(t))
      ROS_WARN("Error Getting Feedback -- Check Connection");

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
