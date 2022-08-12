#pragma once

#include <memory>

#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/trajectory.hpp"

using namespace Eigen;

namespace hebi {

class TreadedBase {
  // FRAME CONVENTION:
  // ORIGIN = MID-POINT BETWEEN THE WHEELS
  // +X-AXIS = FORWARD
  // +Y-AXIS = LEFT
  // +Z-AXIS = UP

  //   Left  |   Right
  //   1     |    2
  //         |
  //         |
  //   3     |    4
public:
  static constexpr float WHEEL_DIAMETER = 0.105f;
  static constexpr float WHEEL_BASE = 0.400f;
  static constexpr float WHEEL_RADIUS = WHEEL_DIAMETER / 2.f;

  static const Eigen::MatrixXd WHEEL_TO_CHASSIS_VEL;
  static const Eigen::MatrixXd CHASSIS_TO_WHEEL_VEL;

  // (Note -- for C++17, use variant here instead)
  // First value of return pair is base.  On error, this is null and error will be returned in second element.
  // Otherwise, ignore second element.
  static std::pair<std::unique_ptr<TreadedBase>, std::string> create(hebi::Lookup& lookup, const std::string& family,
                                                                     std::string gains_file, double t_start);

  bool hasActiveTrajectory() const;
  bool hasActiveFlipperTrajectory() const;
  bool hasActiveBaseTrajectory() const;

  bool isAligning() const;

  bool alignedFlipperMode() const { return aligned_flipper_mode_; }

  void setAlignedFlipperMode(bool value);

  bool flippersAligned() const { return flippersAlignedFront() && flippersAlignedBack(); }

  Eigen::VectorXd alignedFlipperPosition() const;

  void update(double t_now);

  void send() { group_->sendCommand(cmd_); }

  void setFlipperTrajectory(double t_now, double ramp_time, Eigen::VectorXd* p = nullptr, Eigen::VectorXd* v = nullptr);
  void clearFlipperTrajectory() { flipper_traj_= nullptr; }

  void setChassisVelTrajectory(double t_now, double ramp_time, const Eigen::VectorXd& v);
  void clearChassisTrajectory() { chassis_traj_= nullptr; }

  void setColor(hebi::Color color);

  float chassisRampTime() const { return chassis_ramp_time_; }
  float flipperRampTime() const { return flipper_ramp_time_; }

  bool isMStopActive() const { return m_stop_active_; }

private:
  // Only call from "create" with a group which has the gains already set, and initial feedback successfully
  // retrieved.
  TreadedBase(std::shared_ptr<hebi::Group> group, std::unique_ptr<hebi::GroupFeedback> initial_feedback,
              double time_start, float chassis_ramp_time = 0.25f, float flipper_ramp_time = 0.33f)
    : group_(group),
      fbk_(std::move(initial_feedback)),
      cmd_(group_->size()),
      t_prev_(time_start),
      chassis_ramp_time_(chassis_ramp_time),
      flipper_ramp_time_(flipper_ramp_time) {
    cmd_.setPosition(fbk_->getPosition());
  }

  const hebi::Feedback& wheelFeedback(int index) const;
  Eigen::VectorXd wheelFeedbackVelocity() const;
  const hebi::Feedback& flipperFeedback(int index) const;

  void setWheelPositionCommand(const Eigen::VectorXd& pos);
  void incrementWheelPositionCommand(const Eigen::VectorXd& pos_increment);
  void setWheelVelocityCommand(const Eigen::VectorXd& vel);

  void setFlipperPositionCommand(const Eigen::VectorXd& pos);
  void incrementFlipperPositionCommand(const Eigen::VectorXd& pos_increment);
  void setFlipperVelocityCommand(const Eigen::VectorXd& vel);

  void alignFlippers(double* t_now_tmp = nullptr);

  void unlockFlippers() { aligned_flipper_mode_ = false; }

  bool flippersAlignedFront() const;
  bool flippersAlignedBack() const;

  std::shared_ptr<hebi::Group> group_;
  // Note; if this class were moveable, we could do away with the pointer here.
  std::unique_ptr<hebi::GroupFeedback> fbk_;
  hebi::GroupCommand cmd_;

  float chassis_ramp_time_{};
  float flipper_ramp_time_{};

  bool is_aligning_{};
  bool aligned_flipper_mode_{};

  // Note -- start times are a limitation of C++ API v 3.2.0 reducing stability of
  // trajectories with high values of t; need to update and can remove these extra
  // time keepers.
  std::shared_ptr<hebi::trajectory::Trajectory> chassis_traj_;
  double chassis_traj_start_time_{};
  std::shared_ptr<hebi::trajectory::Trajectory> flipper_traj_;
  double flipper_traj_start_time_{};

  double t_prev_;

  // Have we detected a pressed M-Stop on the robot?
  bool m_stop_active_{};
};

} // namespace hebi
