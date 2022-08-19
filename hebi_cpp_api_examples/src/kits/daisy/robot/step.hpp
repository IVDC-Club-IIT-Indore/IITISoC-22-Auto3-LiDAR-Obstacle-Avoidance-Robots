#pragma once

#include "hebi_cpp_api/trajectory.hpp"
#include <Eigen/Dense>

namespace hebi {

class Leg;

// Represents a single step being actively taken by a leg.
class Step
{
public:
  Step(double start_time, const Leg& leg); // NOTE: I don't like this circular dependency on Leg here...
  // TODO: don't call update from constructor?

  // Note: returns 'true' if complete
  bool update(double t);

  const Eigen::Vector3d& getTouchDown() const { return touch_down_; }
  void computeState(double t, Eigen::VectorXd& angles, Eigen::VectorXd& vels, Eigen::VectorXd& accels) const;

  static constexpr float period_ = 0.7f; // seconds
  static constexpr float overshoot_ = 0.3f; // factor of step to overshoot
  static constexpr float height_ = 0.04f; // in meters
  // When creating trajectories, don't use waypoints that are too close together
  static constexpr float ignore_waypoint_threshold_ = 0.01; // 10 ms (in seconds)
  double getStartTime() const { return start_time_; }
private:
  const Leg& leg_;
  // TODO: read from XML -- the phase points of the leg
  static const int num_phase_pts_ = 3;
  const double phase_[num_phase_pts_] = {0, 0.5, 1};
  double start_time_;
  // Time to hit each waypoint
  std::vector<double> time_;

  Eigen::Vector3d lift_off_vel_;

  // Step waypoints: lift up -> high point -> touch down
  Eigen::Vector3d lift_up_;
  Eigen::Vector3d mid_step_1_;
  Eigen::Vector3d mid_step_2_;
  Eigen::Vector3d touch_down_;

  std::shared_ptr<trajectory::Trajectory> trajectory_;
  
  // Allow Eigen member variables:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace hebi
