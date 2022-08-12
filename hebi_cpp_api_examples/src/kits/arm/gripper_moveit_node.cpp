#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <sensor_msgs/JointState.h>

#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/lookup.hpp"

namespace hebi {
namespace ros {

// MoveIt! treats 0 as open and 1.1675 as closed.
// The spool treats -0.9 as closed and 0 as open.
// These functions help convert these quantities.
// Note -- in real systems, cable stretch will result
// in different exact results.

float moveItToSpool(float move_it_pos)
{
  // [0, 1], where 0 is closed and 1 is open.
  auto norm = 1 - (move_it_pos / 1.1675);
  // [-0.9, 0], where -0.9 is closed and 0 is open
  return norm * 0.9 - 0.9;
}

float spoolToMoveIt(float spool_pos)
{
  // Since this is for feedback, and the spool isn't an
  // ideal position transfer source, we scale this a bit
  // more, so ~ -.1 -> 0 and ~-0.66 -> -0.9
  spool_pos += 0.1;
  spool_pos *= 0.9 / (0.66 - 0.1);
  spool_pos = std::min(0.0f, spool_pos);
  spool_pos = std::max(-0.9f, spool_pos);

  // [0, 1], where 0 is closed and 1 is open.
  auto norm = (spool_pos + 0.9) / 0.9;
  // [0, 1.1675], where 0 is open and 1.1675 is closed
  return (1 - norm) * 1.1675;
}

// Scales the velocity feedback
float spoolToMoveItVel(float spool_vel)
{
  // Scale to account for non-ideal position transfer source
  auto slip_scale = spool_vel * 0.9 / (0.66 - 0.1);
  return -slip_scale / 0.9 * 1.1675;
}

// Scales the effort feedback (inverse of velocity scale)
float spoolToMoveItEff(float spool_eff)
{
  // Scale to account for non-ideal position transfer source
  auto slip_scale = spool_eff / 0.9 * (0.66 - 0.1);
  return -slip_scale * 0.9 / 1.1675;
}

// It is designed to use the same interface that MoveIt expects, namely an
// action server for:
// GripperCommand (on hand_controller/gripper_command)
class GripperMoveItNode {
public:
  GripperMoveItNode(hebi::Group& group, ::ros::NodeHandle& node, std::string moveit_joint)
    : group_(group), joint_state_publisher_(node.advertise<sensor_msgs::JointState>("joint_states", 100)) {

    // The "moveit_joint" here is loaded from a rosparam, and should match those in the
    // moveit_config for the arm.
    constexpr size_t num_modules = 1;
    joint_state_message_.name = {moveit_joint};
    joint_state_message_.position.resize(num_modules, 0 );
    joint_state_message_.velocity.resize(num_modules, 0 );
    joint_state_message_.effort.resize(num_modules, 0 );

    group_command_[0].actuator().position().set(0);
  }

  void gripperCommand(const control_msgs::GripperCommandGoalConstPtr& goal) {
    ROS_INFO("Executing follow joint trajectory action");
    // Wait until the action is complete, sending status/feedback along the
    // way.

    /////////////////////////////////
    // Execute command
    /////////////////////////////////

    // Note -- This executes immediately rather than over time.
    group_command_[0].actuator().position().set(moveItToSpool(goal->command.position));

    // Wait until the action is complete, sending status/feedback along the
    // way.
    ::ros::Rate r(10);
    control_msgs::GripperCommandFeedback feedback;

    ROS_INFO("HEBI MoveIt Arm Node executing trajectory");

    if (action_server_->isPreemptRequested() || !::ros::ok()) {
      ROS_INFO("Follow joint trajectory action was preempted");
      // Note -- the `gripperCommand` function will not be called until the
      // action server has been preempted here:
      action_server_->setPreempted();
      return;
    }

    if (!action_server_->isActive() || !::ros::ok()) {
      ROS_INFO("Follow joint trajectory was cancelled");
      return;
    }

    // Publish progress:
    // TODO: we could fill in the action server feedback with desired, actual, and
    // error values if necessary. For now, we rely on the joint_states message to
    // accomplish this.
    // action_server_->publishFeedback(feedback);

    
    // TODO: We could fill in the action server's result with an error code and error string if
    // desired.  Also, we could/should check the path and goal tolerance here.

    // publish when the arm is done with a motion
    ROS_INFO("Completed follow joint trajectory action");
    action_server_->setSucceeded();
  }

  void setActionServer(actionlib::SimpleActionServer<control_msgs::GripperCommandAction>& action_server) {
    action_server_ = &action_server;
  }

  // The "heartbeat" of the program -- sends out messages and updates feedback from
  // and commands to robot
  bool update(::ros::Time t) {
    static int seq = 0;

    // Update position and publish
    group_.sendCommand(group_command_);
    bool res = group_.getNextFeedback(group_feedback_);

    if (res) {
      auto pos = group_feedback_.getPosition();
      auto vel = group_feedback_.getVelocity();
      auto eff = group_feedback_.getEffort();
      for (int i = 0; i < group_feedback_.size(); ++i)
      {
        joint_state_message_.position[i] = spoolToMoveIt(pos[i]);
        joint_state_message_.velocity[i] = spoolToMoveItVel(vel[i]);
        joint_state_message_.effort[i] = spoolToMoveItEff(eff[i]);
      }
      joint_state_message_.header.seq = ++seq;
      joint_state_message_.header.stamp = t;
      joint_state_publisher_.publish(joint_state_message_);
    }
    return res;
  }

private:
  hebi::Group& group_;
  hebi::GroupCommand group_command_{1};
  hebi::GroupFeedback group_feedback_{1};

  actionlib::SimpleActionServer<control_msgs::GripperCommandAction>* action_server_ {nullptr};

  ::ros::Publisher joint_state_publisher_;

  sensor_msgs::JointState joint_state_message_;
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "moveit_gripper_node");
  ros::NodeHandle node;

  /////////////////// Load parameters ///////////////////

  // Get parameters for name/family of modules; default to standard values:
  std::string family;
  if (node.hasParam("family") && node.getParam("family", family)) {
    ROS_INFO("Found and successfully read 'family' parameter");
  } else {
    ROS_WARN("Could not find/read 'family' parameter; defaulting to 'Arm'");
    family = "Arm";
  }

  std::string name;
  if (node.hasParam("name") && node.getParam("name", name)) {
    ROS_INFO("Found and successfully read 'name' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'name' parameter; aborting!");
    return -1;
  }

  std::string moveit_joint;
  if (node.hasParam("moveit_joint") &&
      node.getParam("moveit_joint", moveit_joint) &&
      !moveit_joint.empty()) {
    ROS_INFO("Found and successfully read 'moveit_joint' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'moveit_joint' parameter or parameter was an empty string; aborting!");
    return -1;
  }

  // Read the package + path for the gains file
  std::string gains_package;
  if (node.hasParam("gains_package") && node.getParam("gains_package", gains_package)) {
    ROS_INFO("Found and successfully read 'gains_package' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'gains_package' parameter; aborting!");
    return -1;
  }
  std::string gains_file;
  if (node.hasParam("gains_file") && node.getParam("gains_file", gains_file)) {
    ROS_INFO("Found and successfully read 'gains_file' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'gains_file' parameter; aborting!");
    return -1;
  }

  /////////////////// Initialize gripper ///////////////////

  // Get a group
  std::shared_ptr<hebi::Group> group;
  {
    hebi::Lookup lookup;
    group = lookup.getGroupFromNames({family}, {name}, 10000);
  }

  if (!group) {
    ROS_ERROR("Gripper group not found! Shutting Down...\n");
    return -1;
  }

  // Load the appropriate gains file
  {
    hebi::GroupCommand gains_cmd(group->size());
    if (!gains_cmd.readGains(ros::package::getPath(gains_package) + std::string("/") + gains_file))
      ROS_ERROR("Could not load gripper gains file!");
    else {
      bool success = false;
      for (size_t i = 0; i < 5; ++i) {
        success = group->sendCommandWithAcknowledgement(gains_cmd);
        if (success)
          break;
      }
      if (!success)
        ROS_ERROR("Could not set gripper gains!");
    }
  }

  /////////////////// Initialize ROS interface ///////////////////
 
  hebi::ros::GripperMoveItNode gripper_node(*group, node, moveit_joint);

  // Action server for gripper motions
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_command_action(
    node, "gripper_action",
    boost::bind(&hebi::ros::GripperMoveItNode::gripperCommand, &gripper_node, _1), false);
  gripper_node.setActionServer(gripper_command_action);
  gripper_command_action.start();

  /////////////////// Main Loop ///////////////////

  while (ros::ok()) {

    // Update feedback, and command the gripper
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!gripper_node.update(ros::Time::now()))
      ROS_WARN("Error Getting Feedback -- Check Connection");

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
