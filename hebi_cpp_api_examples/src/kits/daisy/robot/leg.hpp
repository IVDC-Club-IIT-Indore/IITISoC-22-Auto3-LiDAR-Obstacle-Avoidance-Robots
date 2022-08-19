#include <memory>
#include "hebi_cpp_api/robot_model.hpp"
#include "step.hpp"
#include "hexapod_parameters.hpp"

namespace hebi {

class Leg
{
public:
  enum Mode { Stance, Flight };
  enum class LegConfiguration { Left, Right };

  int index_;

  // Transform of leg base from the center of the parent creature.
  Leg(const Matrix4d& base_frame, const Eigen::VectorXd& current_angles, const HexapodParameters& params, bool is_dummy, int index, LegConfiguration configuration);

  // Compute jacobian given position and velocities.  Usually, this is done internally
  // int `computeState`, but if the position/velocity is known (e.g., external
  // step control), this can be used to get these jacobians from the internal
  // kinematics object.
  bool computeJacobians(const Eigen::VectorXd& angles, Eigen::MatrixXd& jacobian_ee, robot_model::MatrixXdVector& jacobian_com) const;
  // TODO: return value?  What if IK fails?
  bool computeState(double t, Eigen::VectorXd& angles, Eigen::VectorXd& vels, Eigen::MatrixXd& jacobian_ee, robot_model::MatrixXdVector& jacobian_com) const;

  // TODO: combine with above computeState?
  // TODO: pass in torques as reference instead for consistency?
  Eigen::VectorXd computeTorques(const robot_model::MatrixXdVector& jacobian_com, const Eigen::MatrixXd& jacobian_ee, const Eigen::VectorXd& angles, const Eigen::VectorXd& vels, const Eigen::Vector3d& gravity_vec, const Eigen::Vector3d& foot_force) const;

  static constexpr int getNumJoints() { return num_joints_; };

  void updateStance(const Eigen::Vector3d& trans_vel, const Eigen::Vector3d& rotate_vel, const Eigen::VectorXd& current_angles, double dt);

  const double getLevelHomeStanceZ() const { return level_home_stance_xyz_(2); }
  const Eigen::Vector3d& getHomeStanceXYZ() const { return home_stance_xyz_; }
  const Eigen::Vector3d& getCmdStanceXYZ() const { return cmd_stance_xyz_; }
  const Eigen::Vector3d& getFbkStanceXYZ() const { return fbk_stance_xyz_; }
  const Eigen::Vector3d& getStanceVelXYZ() const { return stance_vel_xyz_; }

  // NOTE: useful for jumping straight to home stance, e.g. for visualization
  // of stance parameters
  void setCmdStanceToHomeStance() { cmd_stance_xyz_ = home_stance_xyz_; }

  // TODO: think about where this should really be
  const Eigen::VectorXd& getSeedAngles() const { return seed_angles_; }

  // TODO: think about const for this, and other accessor functions for actually
  // getting info from inside
  hebi::robot_model::RobotModel& getKinematics() { return *kin_; }
  const hebi::robot_model::RobotModel& getKinematics() const { return *kin_; }

  // Am I actively stepping?
  Mode getMode() const { return (step_) ? Mode::Flight : Mode::Stance; }

  void startStep(double t);
  void updateStep(double t);
  double getStepTime(double t) const;
  double getStepPeriod() const;

private:

  static constexpr int num_joints_ = 3;
  float stance_radius_; // [m]
  float body_height_; // [m]
  const float spring_shift_; // [N*m] compensate for the spring torques
  Eigen::VectorXd seed_angles_;

  std::unique_ptr<Step> step_;

  Eigen::Vector3d home_stance_xyz_;
  Eigen::Vector3d level_home_stance_xyz_;
  Eigen::Vector3d fbk_stance_xyz_;
  Eigen::Vector3d cmd_stance_xyz_;
  Eigen::Vector3d stance_vel_xyz_;
  
  std::unique_ptr<hebi::robot_model::RobotModel> kin_;
  // Note -- one mass element for each COM frame in the kinematics!
  Eigen::VectorXd masses_;

  // Allow Eigen member variables:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace hebi
