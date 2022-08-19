#include "mobile_io.hpp"

#include "ros/ros.h" // For ROS_WARN

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group_command.hpp"

namespace hebi {

MobileIODiff::MobileIODiff(const MobileIOState& prev, const MobileIOState& curr) {
  for (size_t i = 0; i < NumButtons; ++i) {
    if (prev.buttons_[i] && curr.buttons_[i])
      buttons_[i] = ButtonState::On;
    else if (!prev.buttons_[i] && curr.buttons_[i])
      buttons_[i] = ButtonState::ToOn;
    if (!prev.buttons_[i] && !curr.buttons_[i])
      buttons_[i] = ButtonState::Off;
    else if (prev.buttons_[i] && !curr.buttons_[i])
      buttons_[i] = ButtonState::ToOff;
  }
}

MobileIODiff::ButtonState MobileIODiff::get(int button) {
  if (button < 1 || button > NumButtons) {
    assert(false);
    return ButtonState::Off;
  }
  return buttons_[button - 1];
}

std::unique_ptr<MobileIO> MobileIO::create(const std::string& family, const std::string& name) {
  hebi::Lookup lookup;
  std::shared_ptr<hebi::Group> group;
  while (!group) {
    ROS_WARN("Looking for Mobile IO Controller");
    group = lookup.getGroupFromNames({ family }, { name });
  }
  return std::unique_ptr<MobileIO>(new MobileIO(group));
}

MobileIOState MobileIO::getState() {
  bool tmp;
  return getState(tmp);
}

MobileIOState MobileIO::getState(bool& got_feedback) {
  // Update if we get another packet from the Mobile IO device
  // (on "failure to get data", just return last data)
  got_feedback = false;
  if (group_->getNextFeedback(fbk_)) {
    got_feedback = true;
    // We assume the Mobile IO controller is only ever talking to one
    // device at a time...
    auto& f0 = fbk_[0];
    // Update all the buttons in the current state:
    for (int i = 1; i <= NumButtons; ++i) {
      if (f0.io().b().hasInt(i)) {
        current_state_.buttons_[i - 1] = f0.io().b().getInt(i) == 1;
      }
    }
    // And the axes
    for (int i = 1; i <= NumButtons; ++i) {
      if (f0.io().a().hasFloat(i)) {
        current_state_.axes_[i - 1] = f0.io().a().getFloat(i);
      }
    }
  }
  return current_state_;
}

bool MobileIO::disableSnap(size_t axis_number) {
  return setSnap(axis_number, std::numeric_limits<float>::quiet_NaN());
}

bool MobileIO::setSnap(size_t axis_number, float snap_to) {
  hebi::GroupCommand cmd(group_->size());
  cmd[0].io().a().setFloat(axis_number, snap_to);
  return group_->sendCommand(cmd);
}

bool MobileIO::setAxisValue(size_t axis_number, float value) {
  hebi::GroupCommand cmd(group_->size());
  cmd[0].io().f().setFloat(axis_number, value);
  return group_->sendCommand(cmd);
}

bool MobileIO::setButtonMode(size_t button_number, ButtonMode mode) {
  hebi::GroupCommand cmd(group_->size());
  cmd[0].io().b().setInt(button_number, mode == ButtonMode::Toggle ? 1 : 0);
  return group_->sendCommand(cmd);
}

bool MobileIO::setButtonOutput(size_t button_number, bool on) {
  hebi::GroupCommand cmd(group_->size());
  cmd[0].io().e().setInt(button_number, on ? 1 : 0);
  return group_->sendCommand(cmd);
}
  
bool MobileIO::setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  hebi::GroupCommand cmd(group_->size());
  cmd[0].led().set({r, g, b});
  return group_->sendCommand(cmd);
}

MobileIO::MobileIO(std::shared_ptr<hebi::Group> group)
  : group_(group), fbk_(group_->size())
{ }

} // namespace hebi
