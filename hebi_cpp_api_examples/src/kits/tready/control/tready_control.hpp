#pragma once

#include "robot/treaded_base.hpp" // For constants!

#include "hebi_cpp_api/util/mobile_io.hpp"

namespace hebi {

struct TreadyInputs {
  // Flippers
  Eigen::Vector4d flippers_;
  bool align_flippers_;
  // Chassis Velocity ("base_motion" in python code)
  double base_x_vel_;
  double base_y_vel_;
  double base_rot_vel_;
};

struct TreadyVelocity {
  // Flippers
  Eigen::Vector4d flippers_;
  // Chassis Velocity
  double base_x_vel_;
  // double base_y_vel_;
  double base_rot_vel_;
};

// TODO: constructors and stuff; flesh out
struct DemoInputs {
  bool has_value() const { return has_value_; }

  bool shouldExit() { return false; }  // TODO
  bool shouldReset() { return false; } // TODO

  TreadyInputs chassis_;

private:
  bool has_value_{false};
};

// TODO: constructors and stuff; flesh out
struct BaseState {
  bool hasActiveTrajectory() const { return false; } // TODO
  bool aligned_flipper_mode_{};
  bool is_aligning_{};
};

// TODO: constructors and stuff; flesh out
struct BaseCommand {
  static BaseCommand alignFlippers(bool align_state) { return BaseCommand(); }
  static BaseCommand setFlipperVelTrajectory(Eigen::Vector4d flipper_vel) { return BaseCommand(); }
  static BaseCommand clearFlipperTrajectory() { return BaseCommand(); }
  static BaseCommand setChassisVelTrajectory(Eigen::Vector3d cartesian_vel) { return BaseCommand(); }
  static BaseCommand clearChassisVelTrajectory() { return BaseCommand(); }
  static BaseCommand setColor(const hebi::Color& color) { return BaseCommand(); }
  static BaseCommand homeFlippers() { return BaseCommand(); }
};

// Base class for interfacing with tready control code.  Root state machine used for different operating modes to extend
class TreadyControl {
public:
  enum class DemoState { Startup, Homing, Teleop, Stopped, Exit };

  static constexpr float FLIPPER_VEL_SCALE = 1.f;                                 // rad/sec
  static constexpr float SPEED_MAX_LIN = 0.15f;                                   // m/s
  static constexpr float SPEED_MAX_ROT = SPEED_MAX_LIN / TreadedBase::WHEEL_BASE; // rad/s

  // Track the relevant aspects of the base state; this gets updated when these change (e.g., through a message from the
  // base node)
  void updateBaseState(BaseState base_state);

protected:
  TreadyControl(std::shared_ptr<hebi::experimental::MobileIO> mobile_io, std::function<void(BaseCommand)> base_command_callback)
    : mobile_io_(mobile_io), base_command_callback_(base_command_callback) {
  }

  void setInstructions(std::string message, hebi::Color* color = nullptr);

  virtual void transitionTo(double t_now, DemoState state);

  static TreadyVelocity computeVelocities(const TreadyInputs& chassis_inputs);

  bool update(double t_now, DemoInputs demo_input = {});

  std::shared_ptr<hebi::experimental::MobileIO> mobile_io_;

  // TODO: initial value?
  double mobile_last_fbk_t_{};

  DemoState state_{DemoState::Startup};

  // Keep track of the relevant bits of the base's state here.  Should not be relied on for
  // accurate timing/control flow, just UI, as this may not update instantaneously after setting the state.
  BaseState base_state_;
  std::function<void(BaseCommand)> base_command_callback_;
};

} // namespace hebi