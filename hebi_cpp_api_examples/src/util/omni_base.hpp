#pragma once

#include <memory>

#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/trajectory.hpp"

#include "Eigen/Dense"

/**
 * omni_base.hpp
 *
 * This file provides an implementation in C++ of an omnidirectional base, such as that on
 * HEBI's "Rosie" kit.
 *
 * The current implementation provides code for commanding smooth motions of the base.
 */

namespace hebi {

// TODO: OmniBaseTrajectory is _almost_ identical to ArmTrajectory.  Maybe combine
// these?

// Note: base trajectory doesn't allow for smooth replanning, because that would be...difficult.  It just
// represents relative motion in (x, y, theta)

class OmniBaseTrajectory {
public:
  static OmniBaseTrajectory create(const Eigen::VectorXd& dest_positions, double t_now);

  void getState(double t_now, 
    Eigen::VectorXd& positions, Eigen::VectorXd& velocities, Eigen::VectorXd& accelerations);

  // Plan a trajectory to move with a given velocity for the next couple seconds.
  void replanVel(double t_now, const Eigen::Vector3d& target_vel);

  void replan(
    double t_now,
    const Eigen::MatrixXd& new_positions,
    const Eigen::MatrixXd& new_velocities,
    const Eigen::MatrixXd& new_accelerations);

  void replan(
    double t_now,
    const Eigen::MatrixXd& new_positions);

  // Heuristic to get the timing of the waypoints. This function can be
// modified to add custom waypoint timing.
  Eigen::VectorXd getWaypointTimes(
    const Eigen::MatrixXd& positions,
    const Eigen::MatrixXd& velocities,
    const Eigen::MatrixXd& accelerations);

  std::shared_ptr<hebi::trajectory::Trajectory> getTraj() { return trajectory_; }
  double getTrajStartTime() { return trajectory_start_time_; }
  double getTrajEndTime() { return trajectory_start_time_ + trajectory_->getDuration(); }

private:
  // This is private, because we want to ensure the OmniBaseTrajectory is always
  // initialized correctly after creation; use the "create" factory method
  // instead.
  OmniBaseTrajectory() = default;
      
  std::shared_ptr<hebi::trajectory::Trajectory> trajectory_ {};
  double trajectory_start_time_ {};
};

class OmniBase {
public:
  // Create an omnibase with the given modules.  Initialize the trajectory planner with the given
  // time.
  static std::unique_ptr<OmniBase> create(
    const std::vector<std::string>& families,
    const std::vector<std::string>& names,
    const std::string& gains_file,
    double start_time,
    std::string& error_out);

  // This should be called regularly to ensure commands are sent to the robot.  Recommended
  // calling frequency is ~100Hz. Slow or intermittent calls will result in jerky motion.
  bool update(double time);

  // How far through the last commanded trajectory are we?
  double trajectoryPercentComplete(double time);
  // Has the last commanded trajectory been completed?
  bool isTrajectoryComplete(double time);

  GroupFeedback& getLastFeedback() { return feedback_; }
  OmniBaseTrajectory& getTrajectory() { return base_trajectory_; }

  // Odometry/system state
  const Eigen::Vector3d& getGlobalPose() { return global_pose_; }
  const Eigen::Vector3d& getGlobalVelocity() { return global_vel_; }
  const Eigen::Vector3d& getLocalVelocity() { return local_vel_; }

  // Reset before commanding a new trajectory.
  void resetStart(Color& color);

  void clearColor();

private:
  OmniBase(std::shared_ptr<Group> group,
    OmniBaseTrajectory base_trajectory,
    double start_time);

  void convertSE2ToWheel();
  void updateOdometry(const Eigen::Vector3d& wheel_vel, double dt);

  /* Declare main kinematic variables */
  static constexpr double wheel_radius_ = 0.0762; // m
  static constexpr double base_radius_ = 0.220; // m (center of omni to origin of base)

  std::shared_ptr<Group> group_;

  GroupFeedback feedback_;
  GroupCommand command_;

  // These are just temporary variables to cache output from
  // Trajectory::getState.
  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;
  Eigen::VectorXd accel_;
  Eigen::VectorXd start_wheel_pos_;
  Eigen::VectorXd last_wheel_pos_;
  Eigen::VectorXd wheel_vel_;

  OmniBaseTrajectory base_trajectory_;

  double last_time_{-1};

  // Track odometry; these are updated every time `update` is called.
  // x/y/theta global pose and velocity
  Eigen::Vector3d global_pose_{0, 0, 0};
  Eigen::Vector3d global_vel_{0, 0, 0};
  // Also, local velocity.
  Eigen::Vector3d local_vel_{0, 0, 0};

  // Local (x,y,theta) to wheel velocities
  static Eigen::Matrix3d createJacobian();
  static const Eigen::Matrix3d jacobian_;
  // Wheel velocities to local (x,y,theta)
  static Eigen::Matrix3d createJacobianInv();
  static const Eigen::Matrix3d jacobian_inv_;

  Color color_;
};

}
