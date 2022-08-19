#include "diff_drive.hpp"

namespace hebi {

DiffDriveTrajectory DiffDriveTrajectory::create(const Eigen::VectorXd& dest_positions, double t_now) {
  DiffDriveTrajectory base_trajectory;

  // Set up initial trajectory
  Eigen::MatrixXd positions(2, 1);
  positions.col(0) = dest_positions;
  base_trajectory.replan(t_now, positions);

  return base_trajectory;
}

void DiffDriveTrajectory::getState(
  double t_now, 
  Eigen::VectorXd& positions,
  Eigen::VectorXd& velocities,
  Eigen::VectorXd& accelerations) {

  // (Cap the effective time to the end of the trajectory)
  double t = std::min(t_now - trajectory_start_time_,
                      trajectory_->getDuration());

  trajectory_->getState(t, &positions, &velocities, &accelerations);
}

void DiffDriveTrajectory::replanVels(double t_now, const Eigen::VectorXd& times, const Eigen::MatrixXd& velocities) {
  Eigen::MatrixXd positions(2, 4);
  Eigen::MatrixXd accelerations(2, 4);

  // Copy new waypoints
  auto nan = std::numeric_limits<double>::quiet_NaN();

  positions.col(0).setZero();
  positions.col(1).setConstant(nan);
  positions.col(2).setConstant(nan);
  positions.col(3).setConstant(nan);

  accelerations.col(0).setZero();
  accelerations.col(1).setZero();
  accelerations.col(2).setZero();
  accelerations.col(3).setZero();

  // Create new trajectory
  trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(
                  times, positions, &velocities, &accelerations);
  trajectory_start_time_ = t_now;
}
  
// Updates the Base State by planning a trajectory to a given set of joint
// waypoints.  Uses the current trajectory/state if defined.
// NOTE: this call assumes feedback is populated.
void DiffDriveTrajectory::replan(
  double t_now,
  const Eigen::MatrixXd& new_positions,
  const Eigen::MatrixXd& new_velocities,
  const Eigen::MatrixXd& new_accelerations) {

  int num_joints = new_positions.rows();

  // Start from (0, 0, 0), as this is a relative motion.
  Eigen::VectorXd curr_pos = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_vel = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_accel = Eigen::VectorXd::Zero(num_joints);

  int num_waypoints = new_positions.cols() + 1;

  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);

  // Initial state
  positions.col(0) = curr_pos;
  velocities.col(0) = curr_vel;
  accelerations.col(0) = curr_accel;

  // Copy new waypoints
  positions.rightCols(num_waypoints - 1) = new_positions;
  velocities.rightCols(num_waypoints - 1) = new_velocities;
  accelerations.rightCols(num_waypoints - 1) = new_accelerations;

  // Get waypoint times
  Eigen::VectorXd trajTime =
    getWaypointTimes(positions, velocities, accelerations);

  // Create new trajectory
  trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(
                  trajTime, positions, &velocities, &accelerations);
  trajectory_start_time_ = t_now;
}
  
// Updates the Base State by planning a trajectory to a given set of joint
// waypoints.  Uses the current trajectory/state if defined.
// NOTE: this is a wrapper around the more general replan that
// assumes zero end velocity and acceleration.
void DiffDriveTrajectory::replan(
  double t_now,
  const Eigen::MatrixXd& new_positions) {

  int num_joints = new_positions.rows();
  int num_waypoints = new_positions.cols();

  // Unconstrained velocities and accelerations during the path, but set to
  // zero at the end.
  double nan = std::numeric_limits<double>::quiet_NaN();

  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  velocities.setConstant(nan);
  velocities.rightCols<1>() = Eigen::VectorXd::Zero(num_joints);

  Eigen::MatrixXd accelerations(num_joints, num_waypoints);
  accelerations.setConstant(nan);
  accelerations.rightCols<1>() = Eigen::VectorXd::Zero(num_joints);

  replan(t_now, new_positions, velocities, accelerations);
}

// Heuristic to get the timing of the waypoints. This function can be
// modified to add custom waypoint timing.
Eigen::VectorXd DiffDriveTrajectory::getWaypointTimes(
  const Eigen::MatrixXd& positions,
  const Eigen::MatrixXd& velocities,
  const Eigen::MatrixXd& accelerations) {

  // TODO: make this configurable!
  double rampTime = 0.5; // Per meter
  double dist = std::pow(positions(0, 1) - positions(0, 0), 2) + 
                std::pow(positions(1, 1) - positions(1, 0), 2);
  dist = std::sqrt(dist);

  rampTime *= dist;
  rampTime = std::max(rampTime, 1.25);

  size_t num_waypoints = positions.cols();

  Eigen::VectorXd times(num_waypoints);
  for (size_t i = 0; i < num_waypoints; ++i)
    times[i] = rampTime * (double)i;

  return times;
};    

std::unique_ptr<DiffDrive> DiffDrive::create(
  const std::vector<std::string>& families,
  const std::vector<std::string>& names,
  const std::string& gains_file,
  double start_time,
  std::string& error_out)
{
  // Invalid input!  Size mismatch.  An omnibase has three wheels.
  if (names.size() != 2 || (families.size() != 1 && families.size() != 2)) {
    assert(false);
    return nullptr;
  }

  // Try to find the modules on the network
  Lookup lookup;
  auto group = lookup.getGroupFromNames(families, names, 10000);
  if (!group)
    return nullptr;

  // Load the appropriate gains file.  Try to set 5 times before giving up.
  {
    hebi::GroupCommand gains_cmd(group->size());
    bool success = gains_cmd.readGains(gains_file);
    if (!success)
      error_out = "Could not load base gains file!";
    else
    {
      for (size_t i = 0; i < 5; ++i)
      {
        success = group->sendCommandWithAcknowledgement(gains_cmd);
        if (success)
          break;
      }
      if (!success)
        error_out = "Could not set base gains!";
    }
  }

  constexpr double feedback_frequency = 100;
  group->setFeedbackFrequencyHz(feedback_frequency);
  constexpr long command_lifetime = 250;
  group->setCommandLifetimeMs(command_lifetime);

  // Try to get feedback -- if we don't get a packet in the first N times,
  // something is wrong
  int num_attempts = 0;
  
  // This whole "plan initial trajectory" is a little hokey...but it's better than nothing  
  GroupFeedback feedback(group->size());
  while (!group->getNextFeedback(feedback)) {
    if (num_attempts++ > 20) {
      return nullptr;
    }
  }

  // NOTE: I don't like that start time is _before_ the "get feedback"
  // loop above...but this is only during initialization
  DiffDriveTrajectory base_trajectory = DiffDriveTrajectory::create(Eigen::Vector2d::Zero(), start_time);
  return std::unique_ptr<DiffDrive>(new DiffDrive(group, base_trajectory, feedback, start_time));
}

bool DiffDrive::update(double time) {

  if (!group_->getNextFeedback(feedback_))
    return false;

  // Update command from trajectory
  base_trajectory_.getState(time, pos_, vel_, accel_);
  command_.setPosition(start_wheel_pos_ + pos_);
  command_.setVelocity(vel_);

  for (int i = 0; i < 2; ++i) {
    command_[i].led().set(color_);
  }

  group_->sendCommand(command_);

  return true; 
}
 
double DiffDrive::trajectoryPercentComplete(double time) {
  return std::min((time - base_trajectory_.getTrajStartTime()) / base_trajectory_.getTraj()->getDuration(), 1.0) * 100;
}

bool DiffDrive::isTrajectoryComplete(double time) {
  return time > base_trajectory_.getTrajEndTime();
}
  
void DiffDrive::setColor(Color& color) {
  color_ = color;
}

void DiffDrive::startVelControl(double dx, double dtheta, double time) {
  Eigen::MatrixXd velocities(2, 4);

  // One second to get up to velocity, and then keep going for at least 1 second.
  Eigen::VectorXd times(4);
  times << 0, 0.15, 0.9, 1.2;

  // Initial state

  // Copy new waypoints
  auto nan = std::numeric_limits<double>::quiet_NaN();

  // Get last command to smoothly maintain commanded velocity
  Eigen::VectorXd p(2), v(2), a(2);
  if (base_trajectory_.getTraj()) {
    base_trajectory_.getState(time, p, v, a);
    // The new trajectory starts at 0, so update our offset if
    // based on previous commands
    start_wheel_pos_ += p;
  } else {
    start_wheel_pos_ = feedback_.getPosition();
    p.setZero();
    v.setZero();
    a.setZero();
  }

  Eigen::VectorXd target_vel_wheels(2);

  target_vel_wheels[0] = -1.0 * dtheta * (base_radius_ / wheel_radius_);
  target_vel_wheels[1] = -1.0 * dtheta * (base_radius_ / wheel_radius_);
  // velocity in x to wheel angles
  target_vel_wheels[0] += dx / wheel_radius_;
  target_vel_wheels[1] -= dx / wheel_radius_;
  // y vel is ignored

  velocities.col(0) = v;
  velocities.col(1) = velocities.col(2) = target_vel_wheels;
  velocities.col(3).setZero();

  // Create new trajectory
  base_trajectory_.replanVels(time, times, velocities);
}

void DiffDrive::startRotateBy(float theta, double time) {
  start_wheel_pos_ = feedback_.getPosition();

  // Note: to rotate by 'theta', each wheel needs to move
  // 'base_radius_ * theta'.
  // For a wheel to move 'd', it needs to rotate by
  // 'd / wheel_radius_' radians.
  // So, we need to rotate each wheel by
  // 'base_radius_ * theta / wheel_radius_'
  float wheel_theta = base_radius_ * theta / wheel_radius_;

  // [L; R] final wheel positions
  Eigen::MatrixXd waypoints(2, 1);
  // Because of the way the actuators are mounted, rotating in
  // the '-z' actuator direction will move the robot in a
  // positive robot-frame direction.
  waypoints(0, 0) = -wheel_theta; 
  waypoints(1, 0) = -wheel_theta; 
  base_trajectory_.replan(time, waypoints);
}

void DiffDrive::startMoveForward(float distance, double time) {
  start_wheel_pos_ = feedback_.getPosition();

  // Note: to move by 'distance', each wheel must move
  // together by 'distance / wheel_radius_' radians.

  float wheel_theta = distance / wheel_radius_;
  // [L; R] final wheel positions
  Eigen::MatrixXd waypoints(2, 1);
  // Because of the way the actuators are mounted, we negate
  // the left actuator to move forward.
  waypoints(0, 0) = wheel_theta; 
  waypoints(1, 0) = -wheel_theta; 
  base_trajectory_.replan(time, waypoints);
}

void DiffDrive::clearColor() {
  Color c(0, 0, 0, 0);
  color_ = c;
}
  
DiffDrive::DiffDrive(std::shared_ptr<Group> group,
  DiffDriveTrajectory base_trajectory,
  const GroupFeedback& feedback,
  double start_time)
  : group_(group),
    feedback_(group->size()),
    command_(group->size()),
    pos_(Eigen::VectorXd::Zero(group->size())),
    vel_(Eigen::VectorXd::Zero(group->size())),
    accel_(Eigen::VectorXd::Zero(group->size())),
    start_wheel_pos_(feedback.getPosition()),
    base_trajectory_{base_trajectory}
{
}

}

