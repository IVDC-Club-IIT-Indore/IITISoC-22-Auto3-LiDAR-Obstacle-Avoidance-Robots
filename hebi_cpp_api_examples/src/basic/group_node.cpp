#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <actionlib/server/simple_action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include <hebi_cpp_api_examples/SetCommandLifetime.h>
#include <hebi_cpp_api_examples/SetFeedbackFrequency.h>
#include <hebi_cpp_api_examples/SetGains.h>

#include <hebi_cpp_api_examples/TargetWaypoints.h>

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/trajectory.hpp"

namespace hebi {
namespace ros {

class GroupNode {
public:
  GroupNode(::ros::NodeHandle* nh, const std::shared_ptr<hebi::Group>& group, const std::vector<std::string> link_names, double message_timeout) :
       message_timeout_(message_timeout),
       group_(group),
       pos_(group->size()),
       vel_(group->size()),
       accel_(group->size()),
       command_(group->size()),
       feedback_(group->size()),
       nh_(*nh),
       joint_trajectory_subscriber_(nh->subscribe<trajectory_msgs::JointTrajectory>("joint_waypoints", 50, &GroupNode::updateJointWaypoints, this)),
       joint_point_subscriber_(nh->subscribe<trajectory_msgs::JointTrajectoryPoint>("joint_target", 50, &GroupNode::setJointSetpoint, this)),
       set_lifetime_service_(nh->advertiseService("set_command_lifetime", &GroupNode::setLifetimeCallback, this)),
       set_frequency_service_(nh->advertiseService("set_feedback_frequency", &GroupNode::setFeedbackFrequencyCallback, this)),
       set_gains_(nh->advertiseService("set_gains", &GroupNode::setGainsCallback, this)),
       group_state_pub_(nh->advertise<sensor_msgs::JointState>("joint_states", 50))  {

    state_msg_.name = link_names;
  }

  bool setLifetimeCallback(hebi_cpp_api_examples::SetCommandLifetime::Request& req,
                           hebi_cpp_api_examples::SetCommandLifetime::Response& res) {
    if(req.lifetime.toSec() < 0.0)
      return false;
    return group_->setCommandLifetimeMs(1000.0 * req.lifetime.toSec());
  }

  bool setFeedbackFrequencyCallback(hebi_cpp_api_examples::SetFeedbackFrequency::Request& req,
                                    hebi_cpp_api_examples::SetFeedbackFrequency::Response& res) {
    if(req.frequency_hz < 0.0)
      return false;
    return group_->setFeedbackFrequencyHz(req.frequency_hz);
  }

  bool setGainsCallback(hebi_cpp_api_examples::SetGains::Request& req,
                        hebi_cpp_api_examples::SetGains::Response& res) {
    hebi::GroupCommand gains_cmd(group_->size());
    auto file_path = ::ros::package::getPath(req.gains_package) + std::string("/") + req.gains_file;
    ROS_INFO_STREAM("Loading gains from " << file_path);
    if (!gains_cmd.readGains(file_path))
      return false;
    return group_->sendCommand(gains_cmd);
  }

  void setJointSetpoint(trajectory_msgs::JointTrajectoryPoint joint_setpoint) {
    if (trajectory_) {
      ROS_INFO("Cancelling trajectory, switching to setpoint control");
    }
    trajectory_ = nullptr;
    trajectory_start_time_ = std::numeric_limits<double>::quiet_NaN();
    // record time of last received setpoint (used to clear setpoint if message_timeout is set)
    message_cleared_ = false;
    last_setpoint_time_ = ::ros::Time::now().toSec();

    Eigen::VectorXd pos(group_->size());
    Eigen::VectorXd vel(group_->size());
    Eigen::VectorXd effort(group_->size());

    for(size_t i=0; i < group_->size(); ++i) {
      pos[i] = joint_setpoint.positions[i];
      vel[i] = joint_setpoint.velocities[i];
      effort[i] = joint_setpoint.effort[i];
    }

    command_.setPosition(pos);
    command_.setVelocity(vel);
    command_.setEffort(effort);
  }

  void updateJointWaypoints(trajectory_msgs::JointTrajectory joint_trajectory) {
    auto num_joints = group_->size();
    auto num_waypoints = joint_trajectory.points.size();
    Eigen::MatrixXd pos(num_joints, num_waypoints);
    Eigen::MatrixXd vel(num_joints, num_waypoints);
    Eigen::MatrixXd accel(num_joints, num_waypoints);
    Eigen::VectorXd times(num_waypoints);
    for (size_t waypoint = 0; waypoint < num_waypoints; ++waypoint) {
      auto& cmd_waypoint = joint_trajectory.points[waypoint];

      if (cmd_waypoint.positions.size() != num_joints ||
          cmd_waypoint.velocities.size() != num_joints ||
          cmd_waypoint.accelerations.size() != num_joints) {
        ROS_ERROR_STREAM("Position, velocity, and acceleration sizes not correct for waypoint index " << waypoint);
        return;
      }

      if (cmd_waypoint.effort.size() != 0) {
        ROS_WARN_STREAM("Effort commands in trajectories not supported; ignoring");
      }

      for (size_t joint = 0; joint < num_joints; ++joint) {
        pos(joint, waypoint) = joint_trajectory.points[waypoint].positions[joint];
        vel(joint, waypoint) = joint_trajectory.points[waypoint].velocities[joint];
        accel(joint, waypoint) = joint_trajectory.points[waypoint].accelerations[joint];
      }

      times(waypoint) = cmd_waypoint.time_from_start.toSec();
    }
    updateJointWaypoints(pos, vel, accel, times);
  }

  void publishState() {
    auto& fdbk = feedback_;

    // Need to copy data from VectorXd to vector<double> in ros msg
    auto pos = fdbk.getPosition();
    auto vel = fdbk.getVelocity();
    auto eff = fdbk.getEffort();

    state_msg_.position.resize(pos.size());
    state_msg_.velocity.resize(vel.size());
    state_msg_.effort.resize(eff.size());
    state_msg_.header.stamp = ::ros::Time::now();

    VectorXd::Map(&state_msg_.position[0], pos.size()) = pos;
    VectorXd::Map(&state_msg_.velocity[0], vel.size()) = vel;
    VectorXd::Map(&state_msg_.effort[0], eff.size()) = eff;

    group_state_pub_.publish(state_msg_);
  }

  bool update(double t) {
    if (t < last_time_)
      return false;

    last_time_ = t;

    if (!group_->getNextFeedback(feedback_))
      return false;

    // Update command from trajectory
    if (trajectory_) {
      // (trajectory_start_time_ should not be nan here!)
      double t_traj = ::ros::Time::now().toSec() - trajectory_start_time_;
      t_traj = std::min(t_traj, trajectory_->getDuration());
      trajectory_->getState(t_traj, &pos_, &vel_, &accel_);

      command_.setPosition(pos_);
      command_.setVelocity(vel_);
    } else {
      if (!message_cleared_ && (message_timeout_ > 0) && (t > (last_setpoint_time_ + message_timeout_))) {
        message_cleared_ = true;
        ROS_WARN("No setpoint message received within message_timeout, clearing setpoint.");
        Eigen::VectorXd clear(group_->size());
        auto nan = std::numeric_limits<double>::quiet_NaN();
        for(size_t i=0; i < group_->size(); ++i) {
          clear[i] = nan;
        }
        command_.setPosition(clear);
        command_.setVelocity(clear);
        command_.setEffort(clear);
      }
    }
    return group_->sendCommand(command_);
  }
  
private:

  std::shared_ptr<trajectory::Trajectory> trajectory_;
  double trajectory_start_time_{ std::numeric_limits<double>::quiet_NaN() };
  double last_time_ = 0;
  double last_setpoint_time_ = 0;
  double message_timeout_ = 0;
  bool message_cleared_ = false;
  std::shared_ptr<hebi::Group> group_;
  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;
  Eigen::VectorXd accel_;
  hebi::GroupCommand command_;
  hebi::GroupFeedback feedback_;

  ::ros::NodeHandle nh_;

  sensor_msgs::JointState state_msg_;

  ::ros::Subscriber joint_trajectory_subscriber_;
  ::ros::Subscriber joint_point_subscriber_;
  bool new_setpoint_ = false;

  ::ros::Publisher group_state_pub_;

  ::ros::ServiceServer set_lifetime_service_;
  ::ros::ServiceServer set_frequency_service_;
  ::ros::ServiceServer set_gains_;

  // Each row is a separate joint; each column is a separate waypoint.
  void updateJointWaypoints(const Eigen::MatrixXd& angles,
                            const Eigen::MatrixXd& velocities,
                            const Eigen::MatrixXd& accelerations,
                            const Eigen::VectorXd& times) {
    // Data sanity check:
    if (angles.rows() != velocities.rows()       || // Number of joints
        angles.rows() != accelerations.rows()    ||
        angles.rows() != group_->size() ||
        angles.cols() != velocities.cols()       || // Number of waypoints
        angles.cols() != accelerations.cols()    ||
        angles.cols() != times.size()            ||
        angles.cols() == 0) {
      ROS_ERROR("Angles, velocities, accelerations, or times were not the correct size");
      return;
    }

    int num_joints = angles.rows();

    // If there is a current trajectory, use the commands as a starting point;
    // if not, replan from current feedback.
    Eigen::VectorXd curr_pos = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd curr_vel = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd curr_accel = Eigen::VectorXd::Zero(num_joints);

    // Replan if these is a current trajectory:
    if (trajectory_) {
      double t_traj = last_time_ - trajectory_start_time_;
      t_traj = std::min(t_traj, trajectory_->getDuration());
      trajectory_->getState(t_traj, &curr_pos, &curr_vel, &curr_accel);
    } else {
      curr_pos = feedback_.getPosition();
      curr_vel = feedback_.getVelocity();
      // (accelerations remain zero)
    }

    int num_waypoints = angles.cols() + 1;

    Eigen::MatrixXd new_positions(num_joints, num_waypoints);
    Eigen::MatrixXd new_velocities(num_joints, num_waypoints);
    Eigen::MatrixXd new_accelerations(num_joints, num_waypoints);

    // Initial state
    new_positions.col(0) = curr_pos;
    new_velocities.col(0) = curr_vel;
    new_accelerations.col(0) = curr_accel;

    // Copy new waypoints
    new_positions.rightCols(num_waypoints - 1) = angles;
    new_velocities.rightCols(num_waypoints - 1) = velocities;
    new_accelerations.rightCols(num_waypoints - 1) = accelerations;

    // Get waypoint times
    Eigen::VectorXd waypoint_times(num_waypoints);
    // If time vector is empty, automatically determine times
    if (times.size() == 0) {
      waypoint_times = getWaypointTimes(new_positions, new_velocities, new_accelerations);
    } else {
      waypoint_times(0) = 0;
      waypoint_times.tail(num_waypoints - 1) = times;
    }

    // Create new trajectory
    ROS_INFO("Creating new trajectory");
    trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(
                  waypoint_times, new_positions, &new_velocities, &new_accelerations);
    trajectory_start_time_ = last_time_;
  }

  Eigen::VectorXd getWaypointTimes(
    const Eigen::MatrixXd& positions,
    const Eigen::MatrixXd& velocities,
    const Eigen::MatrixXd& accelerations) {
  
    double rampTime = 1.2;
  
    size_t num_waypoints = positions.cols();
  
    Eigen::VectorXd times(num_waypoints);
    for (size_t i = 0; i < num_waypoints; ++i)
      times[i] = rampTime * (double)i;
  
    return times;
  }
 
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "group_node");
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
  if (node.hasParam("names") && node.getParam("names", names)) {
    ROS_INFO("Found and successfully read 'names' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'names' parameter; aborting!");
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

  double message_timeout;
  if (node.hasParam("message_timeout") && node.getParam("message_timeout", message_timeout)) {
    ROS_INFO("Found and successfully read 'message_timeout' parameter");
  } else {
    message_timeout = 0.0;
  }

  std::shared_ptr<hebi::Group> group;
  for (int num_tries = 0; num_tries < 3; num_tries++) {
    hebi::Lookup lookup;
    group = lookup.getGroupFromNames(families, names);
    if (group) {
      break;
    }
    ROS_WARN("Could not find group actuators, trying again...");
    ros::Duration(1.0).sleep();
  }

  if (!group) {
    ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }

  hebi::GroupCommand gains_cmd(group->size());
  if (!gains_cmd.readGains(::ros::package::getPath(gains_package) + std::string("/") + gains_file)) {
    ROS_ERROR("Could not load gains file. Attempting to continue.");
  } else if (!group->sendCommandWithAcknowledgement(gains_cmd)){
    ROS_ERROR("Could not set group gains. Attempting to continue.");
  }

  /////////////////// Initialize ROS interface ///////////////////

  std::vector<std::string> full_names;
  for (auto name: names) {
    full_names.push_back(families[0] + "/" + name);
  }
   
  hebi::ros::GroupNode group_node(&node, group, full_names, message_timeout);

  /////////////////// Main Loop ///////////////////

  while (::ros::ok()) {
    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!group_node.update(::ros::Time::now().toSec()))
      ROS_WARN("Error Getting Feedback/Sending Commands -- Check Connection");

    group_node.publishState();

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
