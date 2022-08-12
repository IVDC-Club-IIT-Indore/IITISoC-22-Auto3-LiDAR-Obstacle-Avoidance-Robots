#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Inertia.h>

#include "hebi_cpp_api/robot_model.hpp"
#include "hebi_cpp_api/arm/arm.hpp"

namespace arm = hebi::experimental::arm;

namespace hebi {
namespace ros {

// This is a node that translates between ROS commands and the hebi Arm object.
// It is designed to use the same interface that MoveIt expects, namely an
// action server for:
// FollowJointTrajectory (on hebi_arm_controller/follow_joint_trajectory)
class MoveItArmNode {
public:
  MoveItArmNode(arm::Arm& arm, ::ros::NodeHandle& node, std::vector<std::string> joint_names)
    : arm_(arm),
      node_(node),
      action_server_(node, "hebi_arm_controller/follow_joint_trajectory",
                     boost::bind(&MoveItArmNode::receiveJointTrajectory, this, _1),
                     boost::bind(&MoveItArmNode::cancelJointTrajectory, this, _1),
                     false),
      joint_state_publisher_(node.advertise<sensor_msgs::JointState>("joint_states", 100)),
      center_of_mass_publisher_(node.advertise<geometry_msgs::Inertia>("inertia", 100)) {

    auto num_modules = joint_names.size();
    joint_state_message_.name = joint_names;
    joint_state_message_.position.resize(num_modules, 0 );
    joint_state_message_.velocity.resize(num_modules, 0 );
    joint_state_message_.effort.resize(num_modules, 0 );

    action_server_.start();
  }

  void receiveJointTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) {
    next_goal_ = gh;
    has_next_goal_ = true;
  }

  void cancelJointTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) {
    if(trajectory_goal_.getGoalID().id == gh.getGoalID().id) {
      ROS_INFO("Follow joint trajectory was cancelled");
      trajectory_goal_.setCanceled();
      arm_.cancelGoal();
      // Instead of going 'limp', freeze at current position when cancel occurs
      Eigen::VectorXd stop_pos(arm_.size());
      for (size_t i = 0; i < joint_state_message_.position.size(); ++i) {
        stop_pos[i] = joint_state_message_.position[i];
      }
      arm_.setGoal(arm::Goal::createFromPosition(stop_pos));
    } else {
      ROS_WARN("Attempted to cancel trajectory which is NOT the current trajectory");
    }
  }

  void setJointTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
    ROS_INFO("Setting new joint trajectory goal");
    // Builds trajectory to reach goal, and begins execution.

    /////////////////////////////////
    // Remap
    /////////////////////////////////
    
    // We must remap the joint from the ROS message to our J1-J6 ordering for our robot
    auto num_joints = goal->trajectory.points[0].positions.size();
    std::vector<int> ros_to_hebi(num_joints);
    auto& src_names = goal->trajectory.joint_names;
    auto& dst_names = joint_state_message_.name;
    for (int i = 0; i < src_names.size(); ++i)
    {
      auto location = std::find(dst_names.begin(), dst_names.end(), src_names[i]);
      if (location == dst_names.end())
      {
        trajectory_goal_.setAborted();
        return;
      }
      ros_to_hebi[i] = location - dst_names.begin();
    }
    
    /////////////////////////////////
    // Replan trajectory
    /////////////////////////////////

    // Get the positions, velocities, and accelerations to pass into the HEBI
    // trajectory generator. One important note -- we drop the first waypoint here
    // b/c the hebi Arm API adds a waypoint at the current position already.
    auto num_pts = goal->trajectory.points.size();
    Eigen::MatrixXd positions(num_joints, num_pts - 1);
    Eigen::MatrixXd velocities(num_joints, num_pts - 1);
    Eigen::MatrixXd accelerations(num_joints, num_pts - 1);
    Eigen::VectorXd times(num_pts - 1);
    for (int wp = 1; wp < num_pts; ++wp) // Drop the first waypoint! See above comment!
    {
      auto& waypoint = goal->trajectory.points[wp];
      times(wp - 1) = waypoint.time_from_start.toSec();
      for (int j = 0; j < num_joints; ++j)
      {
        auto dest_joint = ros_to_hebi[j];
        positions(dest_joint, wp - 1) = waypoint.positions[j];
        velocities(dest_joint, wp - 1) = waypoint.velocities[j];
        accelerations(dest_joint, wp - 1) = waypoint.accelerations[j];
      }
    }

    auto arm_goal = arm::Goal::createFromWaypoints(times, positions, velocities, accelerations);
    ROS_INFO("Setting Arm Goal");
    arm_.setGoal(arm_goal);
  }

  // The "heartbeat" of the program -- sends out messages and updates feedback from
  // and commands to robot
  // also handles incoming trajectory goals
  bool update(::ros::Time t) {
    static int seq = 0;

    if (has_next_goal_) {
      has_next_goal_ = false;
      if (trajectory_goal_.isValid()) {
        auto curr_status = trajectory_goal_.getGoalStatus().status;
        if(curr_status == actionlib_msgs::GoalStatus::PENDING || curr_status == actionlib_msgs::GoalStatus::ACTIVE) {
          ROS_INFO("New goal received, preempting current trajectory...");
          trajectory_goal_.setCanceled();
        }
      }

      trajectory_goal_ = next_goal_;
      trajectory_goal_.setAccepted();
      setJointTrajectory(trajectory_goal_.getGoal());
    }

    if(trajectory_goal_.isValid()) {
      auto curr_status = trajectory_goal_.getGoalStatus().status;
      if(curr_status == actionlib_msgs::GoalStatus::ACTIVE && arm_.atGoal()) {
        ROS_INFO("Completed follow joint trajectory action");
        trajectory_goal_.setSucceeded();
      }
    }

    // Update arm, and send new commands
    bool res = arm_.update();
    if (res)
      res = arm_.send();

    // Update position and publish

    // Note -- we don't need to reorder here, b/c we are using the HEBI ordering
    // defined when we filled in the message in the MoveItArmNode constructor
    auto& gf = arm_.lastFeedback();
    auto pos = gf.getPosition();
    auto vel = gf.getVelocity();
    auto eff = gf.getEffort();
    for (int i = 0; i < gf.size(); ++i)
    {
      joint_state_message_.position[i] = pos[i];
      joint_state_message_.velocity[i] = vel[i];
      joint_state_message_.effort[i] = eff[i];
    }
    joint_state_message_.header.seq = ++seq;
    joint_state_message_.header.stamp = t;
    joint_state_publisher_.publish(joint_state_message_);

    // compute arm CoM
    auto& model = arm_.robotModel();
    Eigen::VectorXd masses;
    robot_model::Matrix4dVector frames;
    model.getMasses(masses);
    model.getFK(robot_model::FrameType::CenterOfMass, pos, frames);

    center_of_mass_message_.m = 0.0;
    Eigen::Vector3d weighted_sum_com = Eigen::Vector3d::Zero();
    for(int i = 0; i < model.getFrameCount(robot_model::FrameType::CenterOfMass); ++i) {
      center_of_mass_message_.m += masses(i);
      frames[i] *= masses(i);
      weighted_sum_com(0) += frames[i](0, 3);
      weighted_sum_com(1) += frames[i](1, 3);
      weighted_sum_com(2) += frames[i](2, 3);
    }
    weighted_sum_com /= center_of_mass_message_.m;

    center_of_mass_message_.com.x = weighted_sum_com(0);
    center_of_mass_message_.com.y = weighted_sum_com(1);
    center_of_mass_message_.com.z = weighted_sum_com(2);

    center_of_mass_publisher_.publish(center_of_mass_message_);

    return res;
  }

private:
  arm::Arm& arm_;
  ::ros::NodeHandle& node_;

  actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> trajectory_goal_;
  actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> next_goal_;
  bool has_next_goal_ = false;
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;

  ::ros::Publisher joint_state_publisher_;
  ::ros::Publisher center_of_mass_publisher_;

  sensor_msgs::JointState joint_state_message_;
  geometry_msgs::Inertia center_of_mass_message_;
};

} // namespace ros
} // namespace hebi

template <typename T>
bool loadParam(ros::NodeHandle node, std::string varname, T& var) {
  if (node.hasParam(varname) && node.getParam(varname, var)) {
    ROS_INFO_STREAM("Found and successfully read '" << varname << "' parameter");
    return true;
  }

  ROS_ERROR_STREAM("Could not find/read required '" << varname << "' parameter!");
  return false;
}

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "arm_moveit_node");
  ros::NodeHandle node;

  /////////////////// Load parameters ///////////////////

  // Get parameters for name/family of modules; default to standard values:
  std::vector<std::string> families;
  if (node.hasParam("families") && node.getParam("families", families)) {
    ROS_INFO("Found and successfully read 'families' parameter");
  } else {
    ROS_WARN("Could not find/read 'families' parameter; defaulting to 'HEBI'");
    families = {"HEBI"};
  }

  std::vector<std::string> names;
  std::string gains_package;
  std::string gains_file;
  std::string hrdf_package;
  std::string hrdf_file;

  bool success = true;
  success = success && loadParam(node, "names", names);
  success = success && loadParam(node, "gains_package", gains_package);
  success = success && loadParam(node, "gains_file", gains_file);
  success = success && loadParam(node, "hrdf_package", hrdf_package);
  success = success && loadParam(node, "hrdf_file", hrdf_file);

  if(!success) {
    ROS_ERROR("Could not find one or more required parameters; aborting!");
    return -1;
  }

  // Get the "home" position for the arm
  std::vector<double> home_position_vector;
  if (node.hasParam("home_position") && node.getParam("home_position", home_position_vector)) {
    ROS_INFO("Found and successfully read 'home_position' parameter");
  } else {
    ROS_WARN("Could not find/read 'home_position' parameter; defaulting to all zeros!");
  }

  /////////////////// Initialize arm ///////////////////

  // Create arm
  arm::Arm::Params params;
  params.families_ = families;
  params.names_ = names;
  params.hrdf_file_ = ros::package::getPath(hrdf_package) + std::string("/") + hrdf_file;
  params.get_current_time_s_ = []() {
    static double start_time = ros::Time::now().toSec();
    return ros::Time::now().toSec() - start_time;
  }; 

  auto arm = arm::Arm::create(params);
  for (int num_tries = 0; num_tries < 3; num_tries++) {
    arm = arm::Arm::create(params);
    if (arm) {
      break;
    }
    ROS_WARN("Could not initialize arm, trying again...");
    ros::Duration(1.0).sleep();
  }

  if (!arm) {
    ROS_ERROR_STREAM("Failed to find the following modules in family: " << families.at(0));
    for(auto it = names.begin(); it != names.end(); ++it) {
        ROS_ERROR_STREAM("> " << *it);
    }
    ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }

  // Load the appropriate gains file
  if (!arm->loadGains(ros::package::getPath(gains_package) + std::string("/") + gains_file)) {
    ROS_ERROR("Could not load gains file and/or set arm gains. Attempting to continue.");
  }

  // Get the home position, defaulting to (nearly) zero
  Eigen::VectorXd home_position(arm->size());
  if (home_position_vector.empty()) {
    for (size_t i = 0; i < home_position.size(); ++i) {
      home_position[i] = 0.01; // Avoid common singularities by being slightly off from zero
    }
  } else if (home_position_vector.size() != arm->size()) {
    ROS_ERROR("'home_position' parameter not the same length as HRDF file's number of DoF! Aborting!");
    return -1;
  } else {
    for (size_t i = 0; i < home_position.size(); ++i) {
      home_position[i] = home_position_vector[i];
    }
  }

  // Make a list of family/actuator formatted names for the JointState publisher
  std::vector<std::string> full_names;
  for (size_t idx=0; idx<names.size(); ++idx) {
    full_names.push_back(families.at(0) + "/" + names.at(idx));
  }

  /////////////////// Initialize ROS interface ///////////////////

  hebi::ros::MoveItArmNode arm_node(*arm, node, full_names);

  /////////////////// Main Loop ///////////////////

  // We update with a current timestamp so the "setGoal" function
  // is planning from the correct time for a smooth start

  auto t = ros::Time::now();

  arm_node.update(t);
  arm->setGoal(arm::Goal::createFromPosition(home_position));

  auto prev_t = t;
  while (ros::ok()) {
    t = ros::Time::now();

    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm_node.update(t))
      ROS_WARN("Error Getting Feedback -- Check Connection");

    // If a simulator reset has occured, go back to the home position.
    if (t < prev_t) {
      std::cout << "Resetting action server and returning to home pose after simulation reset" << std::endl;
      arm->setGoal(arm::Goal::createFromPosition(home_position));
    }
    prev_t = t;

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
