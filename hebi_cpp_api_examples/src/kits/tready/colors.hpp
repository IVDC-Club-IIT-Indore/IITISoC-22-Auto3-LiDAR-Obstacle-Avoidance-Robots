#pragma once

// Utility functions for selecting common colors.  Should be moved to HEBI C++ API in the future!

#include <hebi_cpp_api/color.hpp>

namespace hebi {
namespace colors {

// Aka module control
inline hebi::Color clear() { return hebi::Color(0, 0, 0, 0); }

// RGB
inline hebi::Color red() { return hebi::Color(255, 0, 0); }
inline hebi::Color green() { return hebi::Color(0, 255, 0); }
inline hebi::Color blue() { return hebi::Color(0, 0, 255); }

// CMY
inline hebi::Color cyan() { return hebi::Color(0, 255, 255); }
inline hebi::Color magenta() { return hebi::Color(255, 0, 255); }
inline hebi::Color yellow() { return hebi::Color(255, 255, 0); }

// B/W
inline hebi::Color black() { return hebi::Color(0, 0, 0); }
inline hebi::Color white() { return hebi::Color(255, 255, 255); }

} // namespace colors

} // namespace hebi