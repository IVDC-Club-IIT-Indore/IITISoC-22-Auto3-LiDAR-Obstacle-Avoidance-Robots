#pragma once

#include <memory>
#include <hebi_cpp_api/group.hpp>
#include "hebi_cpp_api/group_feedback.hpp"

namespace hebi {

static constexpr size_t NumButtons = 8;

// The current state at any time
struct MobileIOState {
  std::array<bool, NumButtons> buttons_;
  std::array<float, NumButtons> axes_;
};

// Difference between two IO states, useful for checking to see if a button 
// has been pressed.
struct MobileIODiff {
  MobileIODiff(const MobileIOState& prev, const MobileIOState& current);

  enum class ButtonState {
    Off, On, // These occur if last + current state are the same
    ToOff, ToOn // Edge triggers; these occur if last + current state are different
  };

  // Note: one-indexed to match buttons on the screen
  ButtonState get(int button);

private:
  std::array<ButtonState, NumButtons> buttons_;
};

// Wrapper around a mobile IO controller
class MobileIO {
public:

  enum class ButtonMode {
    Momentary, Toggle
  };  

  static std::unique_ptr<MobileIO> create(const std::string& family, const std::string& name);

  // Input/feedback
  MobileIOState getState();
  // Input/feedback; the "got_feedback" flag indicates if feedback was received within the timeout
  // or not.
  MobileIOState getState(bool& got_feedback);

  // Outputs
  // Note: one-indexed to match axes/buttons on the screen

  bool disableSnap(size_t axis_number);
  bool setSnap(size_t axis_number, float snap_to);
  bool setAxisValue(size_t axis_number, float value);

  bool setButtonMode(size_t button_number, ButtonMode mode);
  bool setButtonOutput(size_t button_number, bool on);  

  bool setLedColor(uint8_t r, uint8_t g, uint8_t b);

private:
  MobileIO(std::shared_ptr<hebi::Group>);

  std::shared_ptr<hebi::Group> group_;
  hebi::GroupFeedback fbk_;
  MobileIOState current_state_;
};

}


