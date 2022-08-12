#include "arm_teach_repeat_node.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/robot_model.hpp"
#include "hebi_cpp_api/arm/arm.hpp"

namespace arm = hebi::experimental::arm;

namespace hebi {
namespace ros {

// Helper function to get index of name from a path or waypoint list
template <typename NamedItem>
static int getNameIndex(const hebi_cpp_api_examples::Playback& msg, const std::vector<NamedItem>& named_items, const std::string& name_type) {
  int index = msg.index;
  const auto& name = msg.name;
  if (msg.name.size() > 0) {
    auto it = std::find_if(named_items.begin(), named_items.end(), [&name](const NamedItem& item) { return item.name() == name; } );
    if (it != named_items.end()) {
      index = it - named_items.begin();
    } else {
      ROS_ERROR_STREAM("Invalid " << name_type << " name specified; using index instead");
    }
  }

  // Ensure valid index
  if (index < 0 || index >= named_items.size()) {
    ROS_ERROR_STREAM("Invalid " << name_type << " specified;");
    return -1;
  }

  return index;
}

// Helper function to get the index of a name from a waypoint list; this doesn't fall back to the index, and doesn't
// print any error messages
template <typename NamedItem>
static int tryGetNameIndex(const std::string& name, const std::vector<NamedItem>& named_items) {
  int index = -1;
  if (name.size() > 0) {
    auto it = std::find_if(named_items.begin(), named_items.end(), [&name](const NamedItem& item) { return item.name() == name; } );
    if (it != named_items.end()) {
      index = it - named_items.begin();
    }
  }
  return index;
}

void TeachRepeatNode::setCompliantMode(std_msgs::Bool msg) {
  if (msg.data) {
    ROS_INFO("Pausing active command (entering grav comp mode)");
    arm_.cancelGoal();
  } else {
    ROS_INFO("Resuming active command"); 
    auto t = ::ros::Time::now().toSec();
    auto last_position = arm_.lastFeedback().getPosition();
    arm_.setGoal(arm::Goal::createFromPosition(last_position));
    target_xyz_ = arm_.FK(last_position);
  }
}

void TeachRepeatNode::offsetWaypointPlayback(hebi_cpp_api_examples::OffsetPlayback msg) {
  // Only update if target changes!
  if (msg.offset.x == 0 && msg.offset.y == 0 && msg.offset.z == 0)
    return;

  // The last commanded position:
  Eigen::VectorXd position_cmd = arm_.lastFeedback().getPositionCommand();

  // Initialize target from feedback as necessary
  if (!isTargetInitialized())
    target_xyz_ = arm_.FK(position_cmd);

  // Update stored target position:
  target_xyz_.x() += msg.offset.x;
  target_xyz_.y() += msg.offset.y;
  target_xyz_.z() += msg.offset.z;

  // And the offset for the next playback
  waypoint_playback_offset_.x() += msg.offset.x;
  waypoint_playback_offset_.y() += msg.offset.y;
  waypoint_playback_offset_.z() += msg.offset.z;

  // Plan to this point from the last commanded position for smooth motion
  position_cmd = arm_.solveIK(position_cmd, target_xyz_);

  // Replan:
  arm_.setGoal(arm::Goal::createFromPosition(position_cmd));
}

void TeachRepeatNode::saveWaypoint(hebi_cpp_api_examples::SaveWaypoint msg) {
  // Get waypoint
  auto last_fbk_pos = arm_.lastFeedback().getPosition();

  // Name point from message if name is given; default to number of waypoint
  std::string waypoint_name = std::to_string(waypoints_.size() + 1);
  int wp_index = -1;
  if (msg.name.size() > 0)
  {
    waypoint_name = msg.name;
    wp_index = tryGetNameIndex(waypoint_name, waypoints_);
  }
  std::string filename = ::ros::package::getPath("hebi_cpp_api_examples") + "/data/waypoints/" + waypoint_name + ".txt";

  // Add waypoint; save to file
  if (wp_index != -1)
  {
    // If waypoint name matches, overwrite:
    waypoints_[wp_index].setAngles(last_fbk_pos);
    waypoints_[wp_index].write(filename);
  } else {
    waypoints_.emplace_back(waypoint_name, last_fbk_pos);
    waypoints_.back().write(filename);
  }

  ROS_INFO_STREAM("Saving waypoint #" << waypoints_.size());
}

void TeachRepeatNode::startRecordPath(hebi_cpp_api_examples::StartPath msg) {
  if (!currently_constructing_path_.empty()) {
    ROS_ERROR("Restarting new path before finishing old one!");
    currently_constructing_path_.clear(); // Just in case!
  }

  // You could go ahead and call "update" here, and set time to this - path_dt_ time, if you
  // wanted the first waypoint to be immediate.  Waiting provides a bit of time for the
  // input to stabilize after pressing the "start" button, though.
  last_path_point_time_ = ::ros::Time::now().toSec();
  constructing_path_ = true;
}

void TeachRepeatNode::endRecordPath(hebi_cpp_api_examples::EndPath msg) {
  // Finish path
  constructing_path_ = false;

  // Name path from message if name is given; default to number of path
  std::string path_name = std::to_string(paths_.size() + 1);
  int path_index = -1;
  if (msg.name.size() > 0)
  {
    path_name = msg.name;
    path_index = tryGetNameIndex(path_name, paths_);
  }

  std::string filename = ::ros::package::getPath("hebi_cpp_api_examples") + "/data/paths/" + path_name + ".txt";

  // Add path, reset pending path, Save to file
  if (path_index != -1) {
    // If path name matches, overwrite:
    paths_[path_index].setPath(currently_constructing_path_);
    paths_[path_index].write(filename);
  } else {
    paths_.emplace_back(path_name, currently_constructing_path_);
    paths_.back().write(filename);
  }
  currently_constructing_path_.clear();

  ROS_INFO_STREAM("Saving path " << path_name << " with " << paths_.back().size() << " waypoints to file:\n" << filename);
}

void TeachRepeatNode::startPlayback(hebi_cpp_api_examples::Playback msg) {
  if (msg.mode < 0 || msg.mode >= hebi_cpp_api_examples::Playback::NUM_MODES) {
    ROS_ERROR("Invalid playback mode specified!");
    return;
  }

  if (msg.mode == hebi_cpp_api_examples::Playback::GO_TO_WAYPOINT) {
    int wp_index = getNameIndex(msg, waypoints_, "waypoint");
    if (wp_index < 0) {
      return;
    }
    goToWaypoint(waypoints_[wp_index].angles());

  } else {
    int path_index = getNameIndex(msg, paths_, "path");
    if (path_index < 0) {
      return;
    }
    
    if (msg.mode == hebi_cpp_api_examples::Playback::GO_TO_PATH_START) {
      // Offset from initial point
      Eigen::VectorXd path_start = paths_[path_index].waypoint(0);
      Eigen::Vector3d xyz_offset {0.0, 0.0, 0.05};
      waypoint_playback_offset_ = xyz_offset;
      path_start = offsetJoints(path_start, xyz_offset);

      goToWaypoint(path_start);
    } else if (msg.mode == hebi_cpp_api_examples::Playback::PLAY_PATH) {
      playPath(static_cast<size_t>(path_index));
    }
  }
}

void TeachRepeatNode::update(double t) {
  if (constructing_path_ && t >= last_path_point_time_ + path_dt_) {
    currently_constructing_path_.push_back(arm_.lastFeedback().getPosition());
    last_path_point_time_ = t;
  }
}

Eigen::VectorXd TeachRepeatNode::offsetJoints(const Eigen::VectorXd& joint_angles, const Eigen::Vector3d& xyz_offset) const {
  // Do FK to get current point, then offset and do IK:
  Eigen::Vector3d xyz;
  Eigen::Matrix3d orientation;
  arm_.FK(joint_angles, xyz, orientation);

  xyz += xyz_offset;
  return arm_.solveIK(joint_angles, xyz, orientation);
}

void TeachRepeatNode::playPath(size_t path_index) {
  const auto& this_path = paths_[path_index];

  auto num_waypoints = this_path.size();
  auto num_joints = arm_.size();
  Eigen::MatrixXd wp_matrix(num_joints, num_waypoints);
  // Set the time based on the recorded times
  Eigen::VectorXd times(num_waypoints);
  for (size_t w = 0; w < num_waypoints; ++w) {
    // Existing joint angles for waypoint
    auto joints = this_path.waypoint(w);
    joints = offsetJoints(joints, waypoint_playback_offset_); 

    wp_matrix.col(w) = joints;
    times(w) = path_dt_ * (w + 1);
  }
  
  // Update stored target position, based on final joint angles.
  target_xyz_ = arm_.FK(wp_matrix.rightCols<1>());
  waypoint_playback_offset_ = Eigen::Vector3d::Zero();

  auto t = ::ros::Time::now().toSec();
  arm_.setGoal(arm::Goal::createFromPositions(times, wp_matrix));
  ROS_INFO_STREAM("Start path playback through " << num_waypoints << " waypoints");
}

void TeachRepeatNode::goToWaypoint(const Eigen::VectorXd& joint_angles) {
  auto t = ::ros::Time::now().toSec();

  auto joints = joint_angles;
  joints = offsetJoints(joints, waypoint_playback_offset_); 

  target_xyz_ = arm_.FK(joints);
  arm_.setGoal(arm::Goal::createFromPositions(joints));
  ROS_INFO_STREAM("Moving to waypoint");
}

} // namespace ros
} // namespace hebi

// Shared logic to load a parameter into a variable.  Returns "false" if there is a fatal error
// (e.g., could not load a required parameter)
template <typename Param>
bool loadOptionalParam(ros::NodeHandle& node, Param& param, const Param& default_param, const std::string& param_name) {
  if (node.hasParam(param_name) && node.getParam(param_name, param)) {
    ROS_INFO_STREAM("Found and successfully read '" << param_name << "' parameter.");
    return true;
  }

  ROS_WARN_STREAM("Could not find/read '" << param_name << "' parameter; setting default value.");
  param = default_param;
  return false;
}

template <typename Param>
bool loadRequiredParam(ros::NodeHandle& node, Param& param, const std::string& param_name) {
  if (node.hasParam(param_name) && node.getParam(param_name, param)) {
    ROS_INFO_STREAM("Found and successfully read '" << param_name << "' parameter.");
    return true;
  }

  ROS_ERROR_STREAM("Could not find/read required '" << param_name << "' parameter; aborting!");
  return false;
}

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "arm_teach_repeat_node");
  ros::NodeHandle node;

  /////////////////// Load parameters ///////////////////

  // Get parameters for name/family of modules; defaulting to standard values where appropriate
  std::vector<std::string> families, names;
  loadOptionalParam(node, families, { "Arm" }, "families");
  if (!loadRequiredParam(node, names, "names"))
    return 1;

  // Read the package + path for the gains file
  std::string gains_package, gains_file;
  if (!loadRequiredParam(node, gains_package, "gains_package"))
    return 1;
  if (!loadRequiredParam(node, gains_file, "gains_file"))
    return 1;

  // Read the package + path for the hrdf file
  std::string hrdf_package, hrdf_file;
  if (!loadRequiredParam(node, hrdf_package, "hrdf_package"))
    return 1;
  if (!loadRequiredParam(node, hrdf_file, "hrdf_file"))
    return 1;

  // Get the "home" position for the arm
  std::vector<double> home_position_vector;
  loadOptionalParam(node, home_position_vector, {}, "home_position");

  /////////////////// Initialize arm ///////////////////

  // Create arm
  arm::Arm::Params params;
  params.families_ = families;
  params.names_ = names;
  params.hrdf_file_ = ros::package::getPath(hrdf_package) + std::string("/") + hrdf_file;
  params.get_current_time_s_ = []() {
    static double start_time = ros::Time::now().toSec();
    return ros::Time::now().toSec() - start_time;
  };

  auto arm = arm::Arm::create(params);
  for (int num_tries = 0; num_tries < 3; num_tries++) {
    arm = arm::Arm::create(params);
    if (arm) {
      break;
    }
    ROS_WARN("Could not initialize arm, trying again...");
    ros::Duration(1.0).sleep();
  }

  if (!arm) {
    ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }
  
  // Load the appropriate gains file
  if (!arm->loadGains(ros::package::getPath(gains_package) + std::string("/") + gains_file)) {
    ROS_ERROR("Could not load gains file and/or set arm gains. Attempting to continue.");
  }

  // Get the home position, defaulting to (nearly) zero
  Eigen::VectorXd home_position(arm->size());
  if (home_position_vector.empty()) {
    for (size_t i = 0; i < home_position.size(); ++i) {
      home_position[i] = 0.01; // Avoid common singularities by being slightly off from zero
    }
  } else if (home_position_vector.size() != arm->size()) {
    ROS_ERROR("'home_position' parameter not the same length as HRDF file's number of DoF! Aborting!");
    return 1;
  } else {
    for (size_t i = 0; i < home_position.size(); ++i) {
      home_position[i] = home_position_vector[i];
    }
  }

  /////////////////// Initialize ROS interface ///////////////////
   
  hebi::ros::TeachRepeatNode teach_repeat_node(*arm);

  // Load in the named paths and waypoints:
  std::vector<std::string> enabled_paths, enabled_waypoints;
  if (!loadRequiredParam(node, enabled_paths, "enabled_paths") || !loadRequiredParam(node, enabled_waypoints, "enabled_waypoints"))
    return 1;

  size_t num_joints = arm->size();
  ROS_WARN("##############");
  ROS_WARN("Loading Paths:");
  ROS_WARN(" ");
  for (const auto& short_path_name : enabled_paths ) {
    std::string path_name = "data/paths/" + short_path_name;
    std::vector<double> path_pts;
    if (node.hasParam(path_name) && node.getParam(path_name, path_pts) && path_pts.size() % num_joints == 0) {
      teach_repeat_node.addPath(hebi::Path(short_path_name, path_pts, num_joints));
      ROS_WARN_STREAM("Added path \"" << short_path_name << "\" with " << path_pts.size() / num_joints << " points.");
    } else {
      ROS_ERROR_STREAM("Could not load named path parameter or parameter invalid: " << path_name);
    }
  }
  ROS_WARN(" ");
  ROS_WARN("##############");
  ROS_WARN("##############");
  ROS_WARN("Loading Waypoints:");
  ROS_WARN(" ");
  for (const auto& short_waypoint_name : enabled_waypoints ) {
    std::string waypoint_name = "data/waypoints/" + short_waypoint_name;
    std::vector<double> waypoint_pts;
    if (node.hasParam(waypoint_name) && node.getParam(waypoint_name, waypoint_pts) && waypoint_pts.size() == num_joints) {
      teach_repeat_node.addWaypoint(hebi::Waypoint(short_waypoint_name, waypoint_pts));
      ROS_WARN_STREAM("Added waypoint \"" << short_waypoint_name << "\".");
    } else {
      ROS_ERROR_STREAM("Could not load named waypoint parameter or parameter invalid: " << waypoint_name);
    }
  }
  ROS_WARN(" ");
  ROS_WARN("##############");

  // Subscribe to "compliant mode" toggle, so the robot is placed in/out of
  // a "grav comp only" mode.
  ros::Subscriber compliant_mode_subscriber =
    node.subscribe<std_msgs::Bool>("compliant_mode", 50, &hebi::ros::TeachRepeatNode::setCompliantMode, &teach_repeat_node);

  // Subscribe to "teach" commands
  ros::Subscriber save_waypoint_subscriber =
    node.subscribe<hebi_cpp_api_examples::SaveWaypoint>("save_waypoint", 50, &hebi::ros::TeachRepeatNode::saveWaypoint, &teach_repeat_node);
  ros::Subscriber end_path_subscriber = 
    node.subscribe<hebi_cpp_api_examples::EndPath>("end_path", 50, &hebi::ros::TeachRepeatNode::endRecordPath, &teach_repeat_node);

  // Subscribe to "repeat"/playback commands
  ros::Subscriber playback_subscriber =
    node.subscribe<hebi_cpp_api_examples::Playback>("playback", 50, &hebi::ros::TeachRepeatNode::startPlayback, &teach_repeat_node);
  ros::Subscriber start_path_subscriber = 
    node.subscribe<hebi_cpp_api_examples::StartPath>("start_path", 50, &hebi::ros::TeachRepeatNode::startRecordPath, &teach_repeat_node);
  ros::Subscriber offset_playback_subscriber = 
    node.subscribe<hebi_cpp_api_examples::OffsetPlayback>("offset_playback", 50, &hebi::ros::TeachRepeatNode::offsetWaypointPlayback, &teach_repeat_node);

  /////////////////// Main Loop ///////////////////
  //
  // We update with a current timestamp so the "setGoal" function
  // is planning from the correct time for a smooth start

  auto t = ros::Time::now().toSec();

  arm->update();
  teach_repeat_node.update(t);
  arm->setGoal(arm::Goal::createFromPosition(home_position));

  while (ros::ok()) {
    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    t = ros::Time::now().toSec();
    if (!arm->update())
      ROS_WARN("Error Getting Feedback -- Check Connection");
    else if (!arm->send())
      ROS_WARN("Error Sending Commands -- Check Connection");
    teach_repeat_node.update(t);

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
