#include "leg.hpp"
#include <iostream>

namespace hebi {

using ActuatorType = hebi::robot_model::ActuatorType;
using LinkType = hebi::robot_model::LinkType;

Leg::Leg(const Matrix4d& base_frame, const Eigen::VectorXd& current_angles, const HexapodParameters& params, bool is_dummy, int index, LegConfiguration configuration)
  : index_(index), stance_radius_(params.stance_radius_), body_height_(params.default_body_height_), spring_shift_(configuration == LegConfiguration::Right ? 3.75 : -3.75) // Nm
{
  kin_ = configuration == LegConfiguration::Left ?
    hebi::robot_model::RobotModel::loadHRDF(params.resource_path_ + "/left.hrdf") :
    hebi::robot_model::RobotModel::loadHRDF(params.resource_path_ + "/right.hrdf");
  if (!kin_)
  {
    // Could not find HRDF files!
    // TODO: handle this better so we don't segfault later...probably a factory
    // for "leg"
    std::cerr << "Could not find or load HRDF file for leg!" << std::endl;
    std::cerr << "Ensure it is located in: " << params.resource_path_ << std::endl;
    assert("false");
    return;
  }

  kin_->getMasses(masses_);

  kin_->setBaseFrame(base_frame);

  seed_angles_.resize(num_joints_);
  if (configuration == LegConfiguration::Left)
    seed_angles_ << 0.2, -.3, -1.9;
  else
    seed_angles_ << 0.2, .3, 1.9;
 
  Eigen::Vector4d tmp4(stance_radius_, 0, -body_height_, 0);
  home_stance_xyz_ = (base_frame * tmp4).topLeftCorner<3,1>();
  level_home_stance_xyz_ = home_stance_xyz_;

  // Set initial stance position
  Matrix4d end_point_frame;
  kin_->getEndEffector(current_angles, end_point_frame);
  fbk_stance_xyz_ = end_point_frame.topRightCorner<3,1>();
  kin_->getEndEffector(seed_angles_, end_point_frame);
  cmd_stance_xyz_ = end_point_frame.topRightCorner<3,1>();
  // TODO: initialize better here? What did the MATLAB code do? (nevermind -- that fix wasn't
  //cmd_stance_xyz_ = fbk_stance_xyz_;
}

// Compute jacobian given position and velocities
bool Leg::computeJacobians(const Eigen::VectorXd& angles, Eigen::MatrixXd& jacobian_ee, robot_model::MatrixXdVector& jacobian_com) const
{
  kin_->getJEndEffector(angles, jacobian_ee);
  kin_->getJ(robot_model::FrameType::CenterOfMass, angles, jacobian_com);
}
 
bool Leg::computeState(double t, Eigen::VectorXd& angles, Eigen::VectorXd& vels, Eigen::MatrixXd& jacobian_ee, robot_model::MatrixXdVector& jacobian_com) const
{
  // TODO: think about returning an error value, e.g., when IK fails?
  // TODO: add torque!
  angles.resize(num_joints_);
  vels.resize(num_joints_);
  if (step_) // Step
  {
    Eigen::VectorXd accels;
    step_->computeState(t, angles, vels, accels);
    computeJacobians(angles, jacobian_ee, jacobian_com);
  }
  else // Stance
  {
    auto res = kin_->solveIK(
      seed_angles_,
      angles,
      robot_model::EndEffectorPositionObjective(cmd_stance_xyz_));
    if (res.result != HebiStatusSuccess)
    {
      return false;
    }
    computeJacobians(angles, jacobian_ee, jacobian_com);
    // J(1:3,:) \ stance_vel_xyz)
    MatrixXd jacobian_part = jacobian_ee.topLeftCorner(3,jacobian_ee.cols());
    vels = jacobian_part.colPivHouseholderQr().solve(stance_vel_xyz_).eval();
    return true;
  }
}

Eigen::VectorXd Leg::computeTorques(const robot_model::MatrixXdVector& jacobian_com, const Eigen::MatrixXd& jacobian_ee, const Eigen::VectorXd& angles, const Eigen::VectorXd& vels, const Eigen::Vector3d& gravity_vec, const Eigen::Vector3d& foot_force) const
{
  // TODO: pull from XML?
  constexpr float drag_shift = 1.5; // Nm / (rad/sec)
  
  Eigen::VectorXd spring(Leg::getNumJoints());
  spring << 0, spring_shift_ + drag_shift * vels(1), 0;
  Eigen::VectorXd stance(Leg::getNumJoints());
  Eigen::VectorXd grav_comp(Leg::getNumJoints());

  MatrixXd jacobian_part = jacobian_ee.topLeftCorner(3,jacobian_ee.cols());
  stance = jacobian_part.transpose() * (-foot_force);

  grav_comp.setZero();
  for (int i = 0; i < masses_.size(); ++i)
    grav_comp += - jacobian_com[i].topLeftCorner<3,Leg::getNumJoints()>().transpose() * (gravity_vec * masses_[i]);

  return grav_comp + stance + /*dyn_comp + */ spring;
}

void Leg::updateStance(const Eigen::Vector3d& trans_vel, const Eigen::Vector3d& rotate_vel, const Eigen::VectorXd& current_angles, double dt)
{
  // Get linear velocities of stance legs based on rotational/translational
  // velocities
  stance_vel_xyz_ = trans_vel + rotate_vel.cross(cmd_stance_xyz_);

  // Update position
  cmd_stance_xyz_ += trans_vel * dt;
  cmd_stance_xyz_ = (AngleAxisd(rotate_vel(2) * dt, Eigen::Vector3d::UnitZ()) *
                     AngleAxisd(rotate_vel(1) * dt, Eigen::Vector3d::UnitY()) *
                     AngleAxisd(rotate_vel(0) * dt, Eigen::Vector3d::UnitX()) *
                     cmd_stance_xyz_).eval();

  // Update from feedback
  Matrix4d end_point_frame;
  kin_->getEndEffector(current_angles, end_point_frame);
  fbk_stance_xyz_ = end_point_frame.topRightCorner<3,1>();

  // Update home stance to match the current z height
  level_home_stance_xyz_(2) += trans_vel(2) * dt;
  home_stance_xyz_ = AngleAxisd(0.2 * trans_vel(1), Eigen::Vector3d::UnitX()) *
                     AngleAxisd(-0.2 * trans_vel(0), Eigen::Vector3d::UnitY()) *
                     level_home_stance_xyz_;
}

void Leg::startStep(double t)
{
  step_.reset(new Step(t, *this)); // TODO: why not use fbk stance here?
}

void Leg::updateStep(double t)
{
  assert(step_);
  if (!step_)
    return;
  // Update, marking as complete if we finish the step.
  if (step_->update(t))
  {
    cmd_stance_xyz_ = step_->getTouchDown();
    step_.reset(nullptr);
  }
}

double Leg::getStepTime(double t) const
{
  assert(step_);
  if (!step_)
    return 0;
  return t - step_->getStartTime();
}

double Leg::getStepPeriod() const
{
  assert(step_);
  if (!step_)
    return 0;
  return step_->period_;
}

} // namespace hebi
