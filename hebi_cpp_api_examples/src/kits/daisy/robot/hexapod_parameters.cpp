#include "hexapod_parameters.hpp"
#include "xml_util/xml_helpers.hpp"

#include "xml_util/pugixml.hpp"

namespace hebi {

using Matrix4d = Eigen::Matrix4d;

Eigen::Matrix4d HexapodParameters::getLegTransform(int index) const {
  Matrix4d z_rot = Matrix4d::Identity();
  z_rot.topLeftCorner<3,3>() = Eigen::AngleAxisd(M_PI / 180.0f * leg_angle_[index], Eigen::Vector3d::UnitZ()).matrix();

  Matrix4d translate = Matrix4d::Identity();
  translate(0, 3) = leg_offset_[index];

  Matrix4d flip = Matrix4d::Identity();
  if (leg_flip_[index])
    flip.topLeftCorner<3,3>() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();

  return z_rot * flip * translate;
}

void HexapodParameters::resetToDefaults() {
  mass_ = 21.0f;
  stance_radius_ = 0.45f;
  default_body_height_ = 0.12f;
  min_z_ = -0.3f;
  max_z_ = -0.05f;
  max_r_ = 0.18f;
  step_threshold_rotate_ = 0.05f;
  step_threshold_shift_ = 0.02f;
  logging_enabled_ = true;
  low_log_frequency_hz_ = 5;
  high_log_frequency_hz_ = 200;
}

bool HexapodParameters::loadFromFile(const std::string& file) {
  resetToDefaults();

  pugi::xml_document doc;
  auto res = doc.load_file(file.c_str());
  if (!res)
    return false;
  auto root = doc.child("hex_config");
  if (!root)
    return false;

  auto body = root.child("body");
  auto stance = root.child("stance");
  auto step_thresh = root.child("step_threshold");
  auto logging = root.child("logging");
  bool success = true;

  auto legs = body.children();

  int leg_index = 0;
  for (auto leg : body.children("leg")) {
    if (leg_index > 5)
      return false;
    success = success &&
      xml::trySetFloatParameter(leg.attribute("angle"), leg_angle_[leg_index]) &&
      xml::trySetFloatParameter(leg.attribute("offset"), leg_offset_[leg_index]) &&
      xml::trySetBoolParameter(leg.attribute("flip"), leg_flip_[leg_index]);
    ++leg_index;
  }
  if (leg_index != 6)
    return false;

  success = success &&
    xml::trySetFloatParameter(body.attribute("mass"), mass_);
  success = success &&
    xml::trySetFloatParameter(stance.attribute("radius"), stance_radius_) &&
    xml::trySetFloatParameter(stance.attribute("body_height"), default_body_height_) &&
    xml::trySetFloatParameter(stance.attribute("min_foot_height"), min_z_) &&
    xml::trySetFloatParameter(stance.attribute("max_foot_height"), max_z_) &&
    xml::trySetFloatParameter(stance.attribute("max_shift_radius"), max_r_);
  success = success &&
    xml::trySetFloatParameter(step_thresh.attribute("rotate"), step_threshold_rotate_) &&
    xml::trySetFloatParameter(step_thresh.attribute("shift"), step_threshold_shift_);
  success = success &&
    xml::trySetBoolParameter(logging.attribute("enabled"), logging_enabled_) &&
    xml::trySetFloatParameter(logging.attribute("low_frequency_hz"), low_log_frequency_hz_) &&
    xml::trySetFloatParameter(logging.attribute("high_frequency_hz"), high_log_frequency_hz_);

  return success;
}

bool HexapodParameters::saveToFile(const std::string& file) const {
  pugi::xml_document doc;

  auto root = doc.append_child("hex_config");
  root.append_attribute("name") = "config_root";

  auto body = root.append_child("body");
  auto stance = root.append_child("stance");
  auto step_thresh = root.append_child("step_threshold");
  auto logging = root.append_child("logging");
  stance.append_attribute("radius") = stance_radius_;
  stance.append_attribute("body_height") = default_body_height_;
  stance.append_attribute("min_foot_height") = min_z_;
  stance.append_attribute("max_foot_height") = max_z_;
  stance.append_attribute("max_shift_radius") = max_r_;
  body.append_attribute("mass") = mass_;
  for (int leg_index = 0; leg_index < 6; ++leg_index) {
    auto leg = body.append_child("leg");
    leg.append_attribute("angle") = leg_angle_[leg_index];
    leg.append_attribute("offset") = leg_offset_[leg_index];
    leg.append_attribute("flip") = leg_flip_[leg_index];
  }
  step_thresh.append_attribute("rotate") = step_threshold_rotate_;
  step_thresh.append_attribute("shift") = step_threshold_shift_;
  logging.append_attribute("enabled") = logging_enabled_;
  logging.append_attribute("low_frequency_hz") = low_log_frequency_hz_;
  logging.append_attribute("high_frequency_hz") = high_log_frequency_hz_;

  return (doc.save_file(file.c_str()));
}

} // namespace hebi
