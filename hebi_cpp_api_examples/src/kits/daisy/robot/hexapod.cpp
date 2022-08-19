#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/group_info.hpp"
#include "step.hpp"

#include "hexapod.hpp"

#include <chrono>
#include <thread>
#include <ctime>
#include <fstream>

namespace hebi {

std::unique_ptr<Hexapod> Hexapod::create(const HexapodParameters& params, HexapodErrors& hex_errors)
{
  return createPartial(params, {0, 1, 2, 3, 4, 5}, hex_errors);
}

std::unique_ptr<Hexapod> Hexapod::createPartial(const HexapodParameters& params, std::set<int> real_legs, HexapodErrors& hex_errors)
{
  hebi::Lookup lookup;
  std::vector<std::string> names;
  for (int i = 0; i < 6; ++i)
  {
    if (real_legs.count(i) > 0)
    {
      names.push_back("L" + std::to_string(i + 1) + "_J1_base");
      names.push_back("L" + std::to_string(i + 1) + "_J2_shoulder");
      names.push_back("L" + std::to_string(i + 1) + "_J3_elbow");
    }
  }
  std::vector<std::string> family = { "Daisy" };

  long timeout_ms = 4000; // use a 4 second timeout
  auto group = lookup.getGroupFromNames(family, names, timeout_ms);
  if (!group) {
    std::cout
      << "Couldn't find actuators. Looking for:" << std::endl
      << "Family: " << family[0] << std::endl;
    for (auto name : names) {
      std::cout << "Name: \"" << name << "\"" << std::endl;
    }
    std::cout << std::endl << "Actuators in family \"" << family[0] << "\" on network:" << std::endl;
    auto entry_list = lookup.getEntryList();
    for (auto entry : *entry_list) {
      if (entry.family_.compare(family[0]) == 0)
        std::cout << "Name: \"" << entry.name_ << "\"" << std::endl;
    }
    return nullptr;
  }
  group->setCommandLifetimeMs(100);

  // Log everything!
  std::shared_ptr<hebi::Group> log_group_modules;
  std::shared_ptr<hebi::Group> log_group_io;
  if (params.logging_enabled_)
  {
    log_group_io = lookup.getGroupFromNames({"Daisy"}, {"mobileIO"}, timeout_ms);
    log_group_modules = lookup.getGroupFromNames(family, names, timeout_ms);
  }
  return std::unique_ptr<Hexapod>(new Hexapod(group, log_group_io, log_group_modules, params, real_legs, hex_errors));
}

std::unique_ptr<Hexapod> Hexapod::createDummy(const HexapodParameters& params)
{
  HexapodErrors hex_errors;
  return std::unique_ptr<Hexapod>(
    new Hexapod(std::shared_ptr<hebi::Group>(),
                std::shared_ptr<hebi::Group>(),
                std::shared_ptr<hebi::Group>(),
                params, {}, hex_errors));
}

// TODO: think about making this non-const and returning limited values (and/or
// having another function to do this) (or anything that uses this in the main
// function retrieves it from the hexapod)
void Hexapod::updateStance(const Eigen::Vector3d& translation_velocity, const Eigen::Vector3d& rotation_velocity, double dt)
{
  Eigen::Vector3d trans_vel_limited = translation_velocity;
  // Cap Z velocity if we are too high
  double cur_z = legs_[0]->getLevelHomeStanceZ();
  double dz = trans_vel_limited(2) * dt;
  if (cur_z + dz > params_.max_z_)
    trans_vel_limited(2) = (params_.max_z_ - cur_z) / dt;
  if (cur_z + dz < params_.min_z_)
    trans_vel_limited(2) = (params_.min_z_ - cur_z) / dt;

  Eigen::Vector3d rot_vel_limited = rotation_velocity;
  // Don't tilt in step!
  if (mode_ == Mode::Step)
    rot_vel_limited(1) = 0;

  Eigen::VectorXd all_angles(num_angles_);
  {
    std::lock_guard<std::mutex> guard(fbk_lock_);
    all_angles = positions_;
  }
  int num_joints = Leg::getNumJoints();
  Eigen::VectorXd current_leg_angles(num_joints);
  for (int i = 0; i < num_legs_; ++i)
  {
    // subsample from full vector
    current_leg_angles = all_angles.segment(num_joints * i, num_joints);
    // Update stance of leg
    legs_[i]->updateStance(trans_vel_limited, rot_vel_limited, current_leg_angles, dt);
  }
  vel_xyz_ = trans_vel_limited;
}

void Hexapod::setCommand(int leg_index, const VectorXd* angles, const VectorXd* vels, const VectorXd* torques)
{
  int num_joints = Leg::getNumJoints();

  // For fake legs, just update positions vector:
  if (real_legs_.count(leg_index) == 0)
  {
    if (angles != nullptr)
    {
      int leg_offset = leg_index * num_joints;
      std::lock_guard<std::mutex> guard(fbk_lock_);
      positions_.segment(leg_offset, num_joints) = *angles;
    }
    return;
  }
  // Get leg offset, taking into account all legs may not be present. First,
  // count the number of legs before this one:
  int legs_prev = std::count_if(real_legs_.begin(), real_legs_.end(),
    [leg_index](int real_leg_idx) {return real_leg_idx < leg_index; });
  int leg_offset = legs_prev * num_joints;
  if (angles != nullptr)
  {
    assert(angles->size() == num_joints);
    for (int i = 0; i < num_joints; ++i)
      cmd_[leg_offset + i].actuator().position().set((*angles)[i]);
  }
  if (vels != nullptr)
  {
    assert(vels->size() == num_joints);
    for (int i = 0; i < num_joints; ++i)
      cmd_[leg_offset + i].actuator().velocity().set((*vels)[i]);
  }
  if (torques != nullptr)
  {
    assert(torques->size() == num_joints);
    for (int i = 0; i < num_joints; ++i)
      cmd_[leg_offset + i].actuator().effort().set((*torques)[i]);
  }
}

void Hexapod::sendCommand()
{
  if (group_)
    group_->sendCommand(cmd_);
}

bool Hexapod::setGains()
{
  if (!group_)
    return true;

  hebi::GroupCommand gains(group_->size());
  std::string gains_file = params_.resource_path_ + std::string("gains") + std::to_string(group_->size()) + ".xml";
  std::cout << "Loading gains from: " << gains_file << std::endl;
  bool success = gains.readGains(gains_file);
  return success && group_->sendCommandWithAcknowledgement(gains, 4000);
}

void Hexapod::updateMode(bool is_stance)
{
  // This is specialized for two modes:
  if (is_stance)
    mode_ = Mode::Stance;
  else
    mode_ = Mode::Step;
}

Eigen::Matrix4d Hexapod::getBodyPoseFromFeet() const
{
  int num_legs = legs_.size();
  // Create zero-meaned COM values for feet and base
  MatrixXd feet_xyz(3, num_legs);
  MatrixXd base_xyz(3, num_legs);
  for (unsigned int i = 0; i < legs_.size(); ++i)
  {
    if (real_legs_.count(i) == 0)
      feet_xyz.block<3,1>(0,i) = legs_[i]->getCmdStanceXYZ();
    else
      feet_xyz.block<3,1>(0,i) = legs_[i]->getFbkStanceXYZ();
    base_xyz.block<3,1>(0,i) = legs_[i]->getHomeStanceXYZ();
  }
  auto& feet_xyz_com_tmp = feet_xyz.rowwise().mean();
  auto& base_xyz_com_tmp = base_xyz.rowwise().mean();
  Eigen::Vector3d feet_xyz_com = feet_xyz_com_tmp;
  Eigen::Vector3d base_xyz_com = base_xyz_com_tmp;
  // NOTE: I don't think these "eval" expressions are necessary to prevent
  // Eigen aliasing, but better safe than sorry while debugging.
  feet_xyz = (feet_xyz.colwise() - feet_xyz_com).eval();
  base_xyz = (base_xyz.colwise() - base_xyz_com).eval();

  // SVN of weighted corrolation matrix:
  Matrix3d xyz_corr = base_xyz * (feet_xyz.transpose());
  JacobiSVD<MatrixXd> svd(xyz_corr, ComputeThinU | ComputeThinV);
  auto u = svd.matrixU();
  auto v = svd.matrixV();

  // Set scaling matrix to account for reflections
  Matrix3d s = Matrix3d::Identity();
  float tmp = (u * v).determinant();
  s(2,2) = (tmp > 0) - (tmp < 0); // a quick/dirty "sign" implementation

  // Create/return transform
  Matrix4d res = Matrix4d::Identity();
  res.topLeftCorner<3,3>() = v * s * u.transpose();
  res.topRightCorner<3,1>() = feet_xyz_com - base_xyz_com;
  return res;
}

bool Hexapod::needToStep() const
{
  // Return if we are in stance mode or any legs are actively stepping
  if (mode_ == Stance || isStepping())
    return false;

  Eigen::Matrix4d shift_pose = getBodyPoseFromFeet();
  auto stance_shift = shift_pose.topRightCorner<3,1>();

  // Rotated too much
  float yaw = std::abs(std::atan2(shift_pose(1,0), shift_pose(0,0)));
  if (yaw > params_.step_threshold_rotate_)
    return true;

  // Shifted too much in translation
  float sq_pos_norm = stance_shift(0) * stance_shift(0) + stance_shift(1) * stance_shift(1);
  float sq_vel_norm = vel_xyz_(0) * vel_xyz_(0) + vel_xyz_(1) * vel_xyz_(1);
  if ((std::sqrt(sq_pos_norm) + std::sqrt(sq_vel_norm) * Step::period_) > params_.step_threshold_shift_)
    return true;

  return false;
}

bool Hexapod::isStepping() const
{
  for (const auto& leg : legs_) {
    if (leg->getMode() == Leg::Mode::Flight)
      return true;
  }
  return false;
}

// TODO: think about -- can we just call this "StartAlternatingTripod", and have
// other functions to start step patterns, and have everything just work?
void Hexapod::startStep(double t)
{
  // Note -- we should only use this approach to starting a step if not already
  // stepping, because we assume we are taking an alternating tripod step.
  assert(!isStepping());

  // Update which legs should be active
  std::set<int> this_step_legs;
  for (int i = 0; i < num_legs_; ++i)
  {
    if (last_step_legs_.count(i) == 0)
    {
      this_step_legs.insert(i);
      legs_[i]->startStep(t);
    }
  }

  // Save the new starting step state 
  last_step_legs_ = this_step_legs;
}

void Hexapod::updateSteps(double t)
{
  for (auto& leg : legs_)
  {
    // Replan trajectory every timestep for legs in flight
    if (leg->getMode() == Leg::Mode::Flight)
      leg->updateStep(t);
  }
}

Eigen::VectorXd Hexapod::getLastFeedback()
{
  return positions_;
}

Eigen::VectorXd Hexapod::getLegFeedback(int leg_index)
{
  Eigen::VectorXd leg_angles(Leg::getNumJoints());
  int num_joints = Leg::getNumJoints();
  int leg_offset = leg_index * num_joints;

  std::lock_guard<std::mutex> guard(fbk_lock_);

  for (int i = 0; i < num_joints; ++i)
    leg_angles[i] = positions_[leg_offset + i];

  return leg_angles;
}

void Hexapod::computeFootForces(double t, Eigen::MatrixXd& foot_forces) const
{
  Eigen::VectorXd factors(6);
  Eigen::VectorXd blend_factors(6);
  Eigen::Vector3d grav = -gravity_direction_;
  // Get the dot product of gravity with each leg, and then subtract a scaled
  // gravity from the foot stance position.
  // NOTE: Matt is skeptical about this overall approach; but it worked before so we are keeping
  // it for now.
  factors.resize(6); 
  for (int i = 0; i < 6; ++i)
  {
    Eigen::Vector3d stance = legs_[i]->getCmdStanceXYZ();
    double dot_prod = grav.dot(stance);
    factors(i) = (grav * dot_prod - stance).norm();
  }
  double fact_sum = factors.sum();
  for (int i = 0; i < 6; ++i)
    factors(i) = fact_sum / factors(i);
  for (int i = 0; i < 6; ++i)
  {
    // Redistribute weight to just modules in stance
    if (legs_[i]->getMode() == Leg::Mode::Flight)
    {
      double switch_time = 0.1; // XML? time where weight is being shifted to/from a foot
      double curr_step_time = legs_[i]->getStepTime(t);
      double step_period = legs_[i]->getStepPeriod();
      if (curr_step_time < switch_time)
      {
        blend_factors(i) = (switch_time - curr_step_time) / switch_time;
      }
      else if ((step_period - curr_step_time) < switch_time)
      {
        blend_factors(i) = (curr_step_time - (step_period - switch_time)) / switch_time;
      }
      else
      {
        blend_factors(i) = 0;
      }

      factors(i) *= blend_factors(i);
    }
    else
    {
      blend_factors(i) = 1;
    }
  }
  fact_sum = factors.sum();
  factors /= fact_sum;

  // NOTE: here, we have a blend factor for each foot to allow for future gaits;
  // in MATLAB, there was just one scalar for this.  We use "max" here to match
  // the results from MATLAB.
  for (int i = 0; i < 6; ++i)
    factors(i) = factors(i) * (1 + .33 * std::sin(M_PI * blend_factors(i)));

//  std::cout << "factors: " << factors << std::endl;
//  std::cout << "grav: " << grav << std::endl;

  foot_forces.resize(3,6);
  for (int i = 0; i < 6; ++i)
    foot_forces.block<3,1>(0,i) = factors(i) * weight_ * grav;
}

// TODO: make this a class function and remove parameters? Make it better!
// Because we may be using a "partial" hexapod (e.g., some physical module-backed
// legs and some "dummy" legs to allow testing code on a few legs at a time),
// we need to copy from a set of n feedback angles (3 for each leg) into a vector
// of 18 positions (3 for each of the 6 legs) in the combined "dummy" + real leg
// hexapod.
bool copyIntoPositions(Eigen::VectorXd& positions, const hebi::GroupFeedback* fbk, const std::set<int>& real_legs)
{
  bool valid_fbk = true;
  int fbk_idx = 0;
  int num_leg_modules = 3;
  for (int i = 0; i < positions.size();)
  {
    // Not real?  Skip to the next leg!
    if (real_legs.count(std::round(i / num_leg_modules)) == 0)
    {
      i += num_leg_modules;
    }
    else
    {
      auto& pos = (*fbk)[fbk_idx].actuator().position();
      if (pos)
      {
        positions[i] = pos.get();
      }
      else
      {
        valid_fbk = false;
        positions[i] = std::numeric_limits<double>::quiet_NaN();
      }
      fbk_idx++;
      i++;
    }
  }
  return valid_fbk;
}

// Ensure all legs are between -pi/2 and pi/2!
int getFirstOutOfRange(const Eigen::VectorXd& positions)
{
  int num_leg_modules = 3;
  for (int i = 0; i < positions.size(); i+=num_leg_modules)
  {
    if (positions[i] < (-M_PI / 2.0) || positions[i] > (M_PI / 2.0))
    {
      return i / num_leg_modules;
    }
  }
  return -1;
}
    
bool getMStopPressed(const hebi::GroupFeedback& fbk)
{
  for (int i = 0; i < fbk.size(); ++i)
  {
    // At least one module has the m-stop pressed!
    auto& mstop = fbk[i].actuator().mstopState();
    if (mstop.has() && (mstop.get() == hebi::Feedback::MstopState::Triggered))
      return true;
  }
  return false;
}

std::chrono::time_point<std::chrono::steady_clock> Hexapod::getLastFeedbackTime()
{
  std::lock_guard<std::mutex> guard(fbk_lock_);
  return last_fbk;
}

void Hexapod::clearLegColors()
{
  if (!group_)
    return;
  GroupCommand cmd(group_->size());
  for (int i = 0; i < group_->size(); ++i)
    cmd[i].led().set(hebi::Color(0,0,0,0));
  group_->sendCommand(cmd);
}

void Hexapod::setLegColor(int leg_index, uint8_t r, uint8_t g, uint8_t b)
{
  if (!group_)
    return;
  GroupCommand cmd(group_->size());

  // Fancy mapping to allow for partial sets of legs...
  int leg_count = 6;
  int leg_module_start = 0;
  for (int i = 0; i < leg_count; ++i)
  {
    // Only do anything if this is real!
    if (real_legs_.count(i) != 0)
    {
      // This is the leg we want to set:
      if (leg_index == i)
      {
        cmd[leg_module_start].led().set(hebi::Color(r, g, b));
        cmd[leg_module_start + 1].led().set(hebi::Color(r, g, b));
        cmd[leg_module_start + 2].led().set(hebi::Color(r, g, b));
        break;
      }
      leg_module_start += 3;
    }
  }

  group_->sendCommand(cmd);
}

Eigen::Vector3d Hexapod::getGravityDirection()
{
  std::lock_guard<std::mutex> lg(grav_lock_);
  return gravity_direction_;
}

// Note -- because the "cmd_" object is a class member, we have to provide some constructor
// here, and so we just give it a size '1' if there is no group.  This could become a smart
// pointer instead?
Hexapod::Hexapod(std::shared_ptr<Group> group,
                 std::shared_ptr<Group> log_group_input,
                 std::shared_ptr<Group> log_group_modules,
                 const HexapodParameters& params,
                 const std::set<int>& real_legs,
                 HexapodErrors& hex_errors)
 : real_legs_(real_legs), group_(group), log_group_input_(log_group_input), log_group_modules_(log_group_modules), cmd_(group_ ? group_->size() : 1),
   params_(params), mode_(Mode::Step), weight_(params.mass_ * 9.8f)
{
  // TODO: What should the initial dummy position be?
  positions_ = Eigen::VectorXd::Zero(num_angles_);
  hex_errors.has_valid_initial_feedback = true;
  hex_errors.first_out_of_range_leg = -1;
  if (group_)
  {
    hebi::GroupFeedback fbk(group_->size());
    // TODO: ensure feedback? Get before entering function?
    group_->sendFeedbackRequest();  // Potential bug in 1.0.0-rc3 (C lib -rc4) means I have to call this twice, depending on OS.
    group_->getNextFeedback(fbk);
    group_->sendFeedbackRequest();
    hex_errors.has_valid_initial_feedback = group_->getNextFeedback(fbk);
    // Copy data into an array
    hex_errors.has_valid_initial_feedback = copyIntoPositions(positions_, &fbk, real_legs_)
      && hex_errors.has_valid_initial_feedback;
    hex_errors.first_out_of_range_leg = getFirstOutOfRange(positions_);
    hex_errors.m_stop_pressed = getMStopPressed(fbk);
  }

  for (int index = 0; index < 6; ++index) 
    legs_.emplace_back(new Leg(params.getLegTransform(index), getLegFeedback(index), params, real_legs_.count(index)>0, index,
      index % 2 == 0 ? Leg::LegConfiguration::Left : Leg::LegConfiguration::Right));

  // Initialize step information
  last_step_legs_.insert(0);
  last_step_legs_.insert(3);
  last_step_legs_.insert(4);

  // Default to straight down w/ a level chassis
  gravity_direction_ = -Eigen::Vector3d::UnitZ();

  last_fbk = std::chrono::steady_clock::now();
  // Start a background feedback handler
  if (group_)
  {
    group->addFeedbackHandler([this] (const GroupFeedback& fbk)
    {
      // A -z vector in a local frame.
      Eigen::Vector3d down(0, 0, -1);
      Eigen::Vector3d avg_grav;
      avg_grav.setZero();

      std::lock_guard<std::mutex> guard(fbk_lock_);
      last_fbk = std::chrono::steady_clock::now();
      assert(fbk.size() == Leg::getNumJoints() * real_legs_.size());

      // Copy data into an array
      copyIntoPositions(positions_, &fbk, real_legs_);

      // Get averaged body-frame IMU data for each leg
      // TODO: For each, CHECK THIS IS VALID/HAS FEEDBACK!
      int num_leg_joints = Leg::getNumJoints();
      int num_legs_used = std::max((int)real_legs_.size(), 1);
      int leg_count = 0;
      for (int i = 0; i < 6 && leg_count < num_legs_used; ++i)
      {
        if (real_legs_.count(i) == 0)
        {
          continue;
        }
        leg_count++;
        // Get from base of each leg
        int num_prev_legs = std::count_if(real_legs_.begin(), real_legs_.end(), [i](int other_leg) { return other_leg < i; });

        // HEBI Quaternion
        auto mod_orientation = fbk[num_prev_legs * num_leg_joints]
          .imu().orientation().get();
        // Eigen Quaternion
        Eigen::Quaterniond mod_orientation_eig(
          mod_orientation.getW(),
          mod_orientation.getX(),
          mod_orientation.getY(),
          mod_orientation.getZ());
        Eigen::Matrix3d mod_orientation_mat = mod_orientation_eig.toRotationMatrix();

        // Transform
        Eigen::Matrix4d trans = legs_[i]->getKinematics().getBaseFrame();
        Eigen::Vector3d my_grav = trans.topLeftCorner<3,3>() * mod_orientation_mat.transpose() * down;
        // If one of the modules isn't reporting valid feedback, ignore this:
        if (!std::isnan(my_grav[0]) && !std::isnan(my_grav[1]) && !std::isnan(my_grav[2]))
          avg_grav += my_grav;
      }
      // Average the feedback from various modules and normalize.
      avg_grav.normalize();
      {
        std::lock_guard<std::mutex> lg(grav_lock_);
        gravity_direction_ = avg_grav;
      }

      std::chrono::duration<double, std::ratio<1>> dt =
        (std::chrono::steady_clock::now() - this->pose_start_time_);
      this->pose_last_time_ = dt.count();
    });
    group->setFeedbackFrequencyHz(1000.0 / getFeedbackPeriodMs());
  }
}

void Hexapod::startLogging()
{
  // Set up logging if enabled:
  if (log_group_input_ || log_group_modules_)
    std::cout << "Logging to 'logs' directory at " << params_.low_log_frequency_hz_ << "hz with bursts of " << params_.high_log_frequency_hz_ << " hz every 30 minutes." << std::endl;

  std::string log_name_base;
  {
    char standard_file_name[40];
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);
    std::snprintf(standard_file_name, 40, "log_file_%04d-%02d-%02d_%02d.%02d.%02d",
                  1900 + now_tm->tm_year, 1 + now_tm->tm_mon, now_tm->tm_mday, now_tm->tm_hour,
                  now_tm->tm_min, now_tm->tm_sec);
    log_name_base = std::string(standard_file_name);
  }

  if (log_group_input_)
  {
    log_group_input_->setFeedbackFrequencyHz(params_.low_log_frequency_hz_);
    log_group_input_->startLog("logs", log_name_base + "-IO.hebilog");
  }
  if (log_group_modules_)
  {
    log_group_modules_->setFeedbackFrequencyHz(params_.low_log_frequency_hz_);
    log_group_modules_->startLog("logs", log_name_base + "-HEX.hebilog");

  }
}

Hexapod::~Hexapod()
{
  if (group_)
  {
    group_->setFeedbackFrequencyHz(0);
    group_->clearFeedbackHandlers();
  }
  if (log_group_input_)
  {
    log_group_input_->stopLog();
    log_group_input_->setFeedbackFrequencyHz(0);
  }
  if (log_group_modules_)
  {
    log_group_modules_->stopLog();
    log_group_modules_->setFeedbackFrequencyHz(0);
  }
  if (log_group_input_ || log_group_modules_)
    std::cout << "stopped any active logs" << std::endl;
}

} // namespace hebi
