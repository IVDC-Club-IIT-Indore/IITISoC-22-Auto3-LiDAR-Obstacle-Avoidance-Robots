#include "step.hpp"
#include "leg.hpp"

namespace hebi {

Step::Step(double start_time, const Leg& leg)
  : leg_(leg), start_time_(start_time), lift_up_(leg.getCmdStanceXYZ())
{
  for (int i = 0; i < num_phase_pts_; ++i)
    time_.push_back(phase_[i] * period_);

//  update(start_time, leg); // NOTE: I probably don't actually need to call this here, because it gets called from main before accessing the legs.  TODO: try removing?
}

// Note: returns 'true' if complete
bool Step::update(double t)
{
  lift_off_vel_ = leg_.getStanceVelXYZ();

  // Set touch down point to overshoot the stance error; only do this at the beginning to prevent
  // large changes in foot touch down location and corresponding weird trajectories.  TODO: fix to
  // allow small incremental updates to touch down location!
//  std::cout << leg->getHomeStanceXYZ() << std::endl;
  if (t == start_time_)
    touch_down_ = leg_.getHomeStanceXYZ() - (overshoot_ * period_ * lift_off_vel_);

  // Linearly interpolate along ground
//  mid_step_1_ = 0.9 * lift_up_ + 0.1 * touch_down_;
//  mid_step_2_ = 0.25 * lift_up_ + 0.75 * touch_down_;
  mid_step_1_ = 0.5 * lift_up_ + 0.5 * touch_down_;

  // Set z position
//  mid_step_1_(2) += 0.75 * height_;
//  mid_step_2_(2) += height_;
  mid_step_1_(2) += height_;
  double elapsed = t - start_time_;
  // We are done with this step!
  if (elapsed > period_)
    return true;
  // Close enough to the end; don't replan
  else if ((period_ - elapsed) < ignore_waypoint_threshold_)
    return false;

  // Replan based on current waypoints; create new trajectory objects
  // TODO: make this all more modular! (put waypoints in a vector so we can refer to them more easily here?)
  int num_joints = Leg::getNumJoints();
  int max_num_waypoints = 3; // max number of waypoints
  auto& kin = leg_.getKinematics();
 
  VectorXd leg_times(max_num_waypoints); 
  MatrixXd leg_waypoints(num_joints, max_num_waypoints);
  MatrixXd leg_waypoint_vels(num_joints, max_num_waypoints);
  MatrixXd leg_waypoint_accels(num_joints, max_num_waypoints);
  MatrixXd jacobian_ee;
  VectorXd ik_output;

  // Add initial waypoint
  int next_pt = 0;
  if (t == start_time_)// For initial time, add special velocity for lift off:
  {
    kin.solveIK(
      leg_.getSeedAngles(),
      ik_output,
      robot_model::EndEffectorPositionObjective(lift_up_));
    if (ik_output.size() == 0)
      assert(false);

    // J(1:3, :) \ lift_off_vel;
    kin.getJEndEffector(ik_output, jacobian_ee);
    MatrixXd jacobian_part = jacobian_ee.topLeftCorner(3,jacobian_ee.cols());
    leg_waypoint_vels.block(0, next_pt, num_joints, 1) =
      jacobian_part.colPivHouseholderQr().solve(lift_off_vel_).eval().cast<double>();

    leg_waypoints.col(next_pt) = ik_output;
    leg_waypoint_accels.col(next_pt).setZero();
    
    leg_times[next_pt] = time_[0];
    next_pt++;
  }
  else // If not initial time, add our current point:
  {
    Eigen::VectorXd p(num_joints), v(num_joints), a(num_joints);
    trajectory_->getState(elapsed, &p, &v, &a);

    // Note -- we will definately have past trajectory here, because it is
    // created above when (t == start_time_).
    // TODO: maybe use current position and velocity instead of trajectory ones?
    leg_waypoints.col(next_pt) = p;
    leg_waypoint_vels.col(next_pt) = v;
    leg_waypoint_accels.col(next_pt) = a;

    leg_times[next_pt] = elapsed;
    next_pt++;
  }

  // Now, add remaining waypoints
  if ((time_[1] - elapsed) > ignore_waypoint_threshold_)
  {
    kin.solveIK(
      leg_.getSeedAngles(),
      ik_output,
      robot_model::EndEffectorPositionObjective(mid_step_1_));
    leg_waypoints.col(next_pt) = ik_output;
    leg_waypoint_vels.col(next_pt).setConstant(std::numeric_limits<double>::quiet_NaN());
    leg_waypoint_accels.col(next_pt).setConstant(std::numeric_limits<double>::quiet_NaN());
    leg_times[next_pt] = time_[1];
    next_pt++;
  }
  if ((time_[2] - elapsed) > ignore_waypoint_threshold_)
  {
    kin.solveIK(
      leg_.getSeedAngles(),
      ik_output,
      robot_model::EndEffectorPositionObjective(touch_down_));
    // J(1:3, :) \ stance_vel;
    kin.getJEndEffector(ik_output, jacobian_ee);
    MatrixXd jacobian_part = jacobian_ee.topLeftCorner(3,jacobian_ee.cols());
    leg_waypoint_vels.block(0, next_pt, num_joints, 1) =
      jacobian_part.colPivHouseholderQr().solve(leg_.getStanceVelXYZ()).eval().cast<double>();

    leg_waypoints.col(next_pt) = ik_output;
    leg_waypoint_accels.col(next_pt).setZero();
    leg_times[next_pt] = time_[2];
    next_pt++;
  }
/*
  if ((time_[3] - elapsed) > ignore_waypoint_threshold_)
  {
    kin.solveIK(touch_down_, leg_.getSeedAngles(), ik_output);
    // TODO: get Jacobian, use this to get final vel.
    leg_waypoints.col(next_pt) = ik_output;
    leg_waypoint_vels.col(next_pt).setZero();
    leg_waypoint_accels.col(next_pt).setConstant(std::numeric_limits<double>::quiet_NaN());
    leg_times.push_back(time_[3]);
    next_pt++;
  }
*/
  int num_pts = next_pt;
  // TODO: don't resize, but just be smarter about passing in partial vectors
  // or creating the right size in the first place!
  leg_waypoint_vels.conservativeResize(num_joints, num_pts);
  leg_waypoint_accels.conservativeResize(num_joints, num_pts);

  trajectory_ = trajectory::Trajectory::createUnconstrainedQp(
    leg_times.head(num_pts),
    leg_waypoints.topLeftCorner(num_joints, num_pts),
    &leg_waypoint_vels,
    &leg_waypoint_accels);

  assert(trajectory_);
  return false; // Not done with the step
}
  
void Step::computeState(double t, Eigen::VectorXd& angles, Eigen::VectorXd& vels, Eigen::VectorXd& accels) const
{
  assert(trajectory_);
  if (!trajectory_)
    return;
  int num_joints = Leg::getNumJoints();
  Eigen::VectorXd p(num_joints), v(num_joints), a(num_joints);
  bool success = trajectory_->getState(t - start_time_, &angles, &v, &a);
  assert(success);
  if (success)
    vels = v;
//    vels = v.cast<float>();
}

} // namespace hebi
