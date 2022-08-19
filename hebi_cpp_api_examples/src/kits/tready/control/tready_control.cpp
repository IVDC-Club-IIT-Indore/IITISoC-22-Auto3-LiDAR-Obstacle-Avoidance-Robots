#include "tready_control.hpp"

#include "colors.hpp"

namespace hebi {

void TreadyControl::setInstructions(std::string message, hebi::Color* color) {
  mobile_io_->clearText();
  mobile_io_->sendText(message);
  if (color)
    mobile_io_->setLedColor(*color);
}

void TreadyControl::transitionTo(double t_now, DemoState state) {
  // self transitions are noop
  if (state_ == state)
    return;

  if (state == DemoState::Homing) {
    // print("TRANSITIONING TO HOMING")
    base_command_callback_(BaseCommand::setColor(hebi::colors::magenta()));
    setInstructions("Robot Homing Sequence\nPlease wait...");
    base_command_callback_(BaseCommand::homeFlippers());
  } else if (state == DemoState::Teleop) {
    // print("TRANSITIONING TO TELEOP")
    base_command_callback_(BaseCommand::setColor(hebi::colors::clear()));
    auto color = hebi::colors::green();
    setInstructions("Robot Ready to Control\nB1: Reset\nB6: Joined Flipper\nB8 - Quit", &color);
  } else if (state == DemoState::Stopped) {
    // print("TRANSITIONING TO STOPPED")
    base_command_callback_(BaseCommand::clearChassisVelTrajectory());
    base_command_callback_(BaseCommand::clearFlipperTrajectory());
    base_command_callback_(BaseCommand::setColor(hebi::colors::blue()));
  } else if (state == DemoState::Exit) {
    // print("TRANSITIONING TO EXIT")
    base_command_callback_(BaseCommand::setColor(hebi::colors::red()));
    // unset mobileIO control config
    mobile_io_->setButtonMode(6, experimental::MobileIO::ButtonMode::Momentary);
    mobile_io_->setButtonOutput(1, 0);
    mobile_io_->setButtonOutput(8, 0);
    auto color = hebi::colors::red();
    setInstructions("Demo Stopped", &color);
  }
  state_ = state;
}

TreadyVelocity TreadyControl::computeVelocities(const TreadyInputs& chassis_inputs) {
  // Flipper Control
  auto flip1 = chassis_inputs.flippers_[0];
  auto flip2 = chassis_inputs.flippers_[1];
  auto flip3 = chassis_inputs.flippers_[2];
  auto flip4 = chassis_inputs.flippers_[3];

  TreadyVelocity res;
  auto sign = [](const double& a) { return a >= 0 ? 1.f : -1.f; };

  if (chassis_inputs.align_flippers_) {
    res.flippers_[0] =
        std::max(std::abs(flip1), std::abs(flip2)) * sign(flip1 + flip2) * TreadyControl::FLIPPER_VEL_SCALE;
    res.flippers_[1] = -res.flippers_[0];
    res.flippers_[2] =
        std::max(std::abs(flip3), std::abs(flip4)) * sign(flip3 + flip4) * TreadyControl::FLIPPER_VEL_SCALE;
    res.flippers_[3] = -res.flippers_[2];
  } else {
    res.flippers_[0] = flip1 * TreadyControl::FLIPPER_VEL_SCALE;
    res.flippers_[1] = -flip2 * TreadyControl::FLIPPER_VEL_SCALE;
    res.flippers_[2] = flip3 * TreadyControl::FLIPPER_VEL_SCALE;
    res.flippers_[3] = -flip4 * TreadyControl::FLIPPER_VEL_SCALE;
  }

  // Mobile Base Control
  res.base_x_vel_ = TreadyControl::SPEED_MAX_LIN * chassis_inputs.base_x_vel_;
  res.base_rot_vel_ = TreadyControl::SPEED_MAX_ROT * chassis_inputs.base_rot_vel_;

  return res;
}

bool TreadyControl::update(double t_now, DemoInputs demo_input) {

  if (state_ == DemoState::Exit)
    return false;

  if (demo_input.has_value()) {
    if (t_now - mobile_last_fbk_t_ > 1.0) {
      // print("mobileIO timeout, disabling motion");
      transitionTo(t_now, DemoState::Stopped);
    }
    return true;
  } else {
    mobile_last_fbk_t_ = t_now;

    if (demo_input.shouldExit())
      transitionTo(t_now, DemoState::Exit);
    else if (demo_input.shouldReset())
      transitionTo(t_now, DemoState::Homing);

    if (state_ == DemoState::Stopped) {
      mobile_last_fbk_t_ = t_now;
      transitionTo(t_now, DemoState::Teleop);
      return true;
    } else if (state_ == DemoState::Homing) {
      if (!base_state_.hasActiveTrajectory())
        transitionTo(t_now, DemoState::Teleop);
      return true;
    }

    else if (state_ == DemoState::Teleop) {
      auto desired_flipper_mode = demo_input.chassis_.align_flippers_;
      if (base_state_.aligned_flipper_mode_ != desired_flipper_mode)
        base_command_callback_(BaseCommand::alignFlippers(desired_flipper_mode));
      else if (!base_state_.is_aligning_) {
        // only accept new base commands if flippers are not aligning
        auto vels = computeVelocities(demo_input.chassis_);
        Eigen::Vector3d chassis_vels;
        chassis_vels[0] = vels.base_x_vel_;
        chassis_vels[1] = 0.;
        chassis_vels[2] = vels.base_rot_vel_;
        base_command_callback_(BaseCommand::setChassisVelTrajectory(chassis_vels));
        base_command_callback_(BaseCommand::setFlipperVelTrajectory(vels.flippers_));
      }
      return true;
    }

    else if (state_ == DemoState::Startup) {
      transitionTo(t_now, DemoState::Homing);
      return true;
    }
  }
  return false;
}

} // namespace hebi
