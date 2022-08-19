#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Point.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Inertia.h>
#include <std_srvs/SetBool.h>

#include <hebi_cpp_api_examples/TargetWaypoints.h>
#include <hebi_cpp_api_examples/ArmMotionAction.h>
#include <hebi_cpp_api_examples/SetIKSeed.h>

#include "hebi_cpp_api/group_command.hpp"

#include "hebi_cpp_api/robot_model.hpp"

#include "hebi_cpp_api/arm/arm.hpp"

namespace arm = hebi::experimental::arm;

namespace hebi {
namespace ros {

class ArmNode {
public:
  ArmNode(::ros::NodeHandle* nh, arm::Arm& arm, const Eigen::VectorXd& home_position, std::vector<std::string> link_names) : nh_(*nh),
       arm_(arm),
       home_position_(home_position),
       action_server_(*nh, "motion", boost::bind(&ArmNode::startArmMotion, this, _1), false),
       offset_target_subscriber_(nh->subscribe<geometry_msgs::Point>("offset_target", 50, &ArmNode::offsetTargetCallback, this)),
       set_target_subscriber_(nh->subscribe<geometry_msgs::Point>("set_target", 50, &ArmNode::setTargetCallback, this)),
       cartesian_waypoint_subscriber_(nh->subscribe<hebi_cpp_api_examples::TargetWaypoints>("cartesian_waypoints", 50, &hebi::ros::ArmNode::updateCartesianWaypoints, this)),
       compliant_mode_service_(nh->advertiseService("compliance_mode", &ArmNode::setCompliantMode, this)),
       ik_seed_service_(nh->advertiseService("set_ik_seed", &hebi::ros::ArmNode::handleIKSeedService, this)),
       joint_waypoint_subscriber_(nh->subscribe<trajectory_msgs::JointTrajectory>("joint_waypoints", 50, &ArmNode::updateJointWaypoints, this)),
       arm_state_pub_(nh->advertise<sensor_msgs::JointState>("joint_states", 50)),
       center_of_mass_publisher_(nh->advertise<geometry_msgs::Inertia>("inertia", 100)) {

    //TODO: Figure out a way to get link names from the arm, so it doesn't need to be input separately
    state_msg_.name = link_names;

    // start the action server
    action_server_.start();
  }

  bool setIKSeed(const std::vector<double>& ik_seed) {
    if(ik_seed.size() != home_position_.size()) {
      use_ik_seed_ = false;
    } else {
      use_ik_seed_ = true;
      ik_seed_.resize(ik_seed.size());
      for (size_t i = 0; i < ik_seed.size(); ++i) {
        ik_seed_[i] = ik_seed[i];
      }
    }
    return use_ik_seed_;
  }

  bool handleIKSeedService(hebi_cpp_api_examples::SetIKSeed::Request& req, hebi_cpp_api_examples::SetIKSeed::Response& res) {
    return setIKSeed(req.seed);
  }

  // Callback for trajectories with joint angle waypoints
  void updateJointWaypoints(trajectory_msgs::JointTrajectory joint_trajectory) {
    auto num_joints = arm_.size();
    auto num_waypoints = joint_trajectory.points.size();
    Eigen::MatrixXd pos(num_joints, num_waypoints);
    Eigen::MatrixXd vel(num_joints, num_waypoints);
    Eigen::MatrixXd accel(num_joints, num_waypoints);
    Eigen::VectorXd times(num_waypoints);
    for (size_t waypoint = 0; waypoint < num_waypoints; ++waypoint) {
      auto& cmd_waypoint = joint_trajectory.points[waypoint];

      if (cmd_waypoint.positions.size() != num_joints ||
          cmd_waypoint.velocities.size() != num_joints ||
          cmd_waypoint.accelerations.size() != num_joints) {
        ROS_ERROR_STREAM("Position, velocity, and acceleration sizes not correct for waypoint index " << waypoint);
        return;
      }

      if (cmd_waypoint.effort.size() != 0) {
        ROS_WARN_STREAM("Effort commands in trajectories not supported; ignoring");
      }

      for (size_t joint = 0; joint < num_joints; ++joint) {
        pos(joint, waypoint) = joint_trajectory.points[waypoint].positions[joint];
        vel(joint, waypoint) = joint_trajectory.points[waypoint].velocities[joint];
        accel(joint, waypoint) = joint_trajectory.points[waypoint].accelerations[joint];
      }

      times(waypoint) = cmd_waypoint.time_from_start.toSec();
    }
    updateJointWaypoints(pos, vel, accel, times);
  }

  void updateCartesianWaypoints(hebi_cpp_api_examples::TargetWaypoints target_waypoints) {
    if (action_server_.isActive())
      action_server_.setAborted();

    // Fill in an Eigen::Matrix3xd with the xyz goal
    size_t num_waypoints = target_waypoints.waypoints_vector.size();
    Eigen::Matrix3Xd xyz_waypoints(3, num_waypoints);
    for (size_t i = 0; i < num_waypoints; ++i) {
      const auto& xyz_waypoint = target_waypoints.waypoints_vector[i];
      xyz_waypoints(0, i) = xyz_waypoint.x;
      xyz_waypoints(1, i) = xyz_waypoint.y;
      xyz_waypoints(2, i) = xyz_waypoint.z;
    }

    // Replan
    updateCartesianWaypoints(xyz_waypoints);
  }

  // Set the target end effector location in (x,y,z) space, replanning
  // smoothly to the new location
  void setTargetCallback(geometry_msgs::Point data) {
    if (action_server_.isActive())
      action_server_.setAborted();

    // Fill in an Eigen::Matrix3xd with the xyz goal
    Eigen::Matrix3Xd xyz_waypoints(3, 1);
    xyz_waypoints(0, 0) = data.x;
    xyz_waypoints(1, 0) = data.y;
    xyz_waypoints(2, 0) = data.z;

    // Replan
    updateCartesianWaypoints(xyz_waypoints);
  }

  // "Jog" the target end effector location in (x,y,z) space, replanning
  // smoothly to the new location
  void offsetTargetCallback(geometry_msgs::Point data) {
    if (action_server_.isActive())
      action_server_.setAborted();

    // Only update if target changes!
    if (data.x == 0 && data.y == 0 && data.z == 0)
      return;

    // Initialize target from feedback as necessary
    if (!isTargetInitialized()) {
      auto pos = arm_.FK(arm_.lastFeedback().getPositionCommand());
      target_xyz_.x() = pos.x();
      target_xyz_.y() = pos.y();
      target_xyz_.z() = pos.z();
    }

    // Fill in an Eigen::Matrix3xd with the xyz goal
    Eigen::Matrix3Xd xyz_waypoints(3, 1);
    xyz_waypoints(0, 0) = target_xyz_.x() + data.x;
    xyz_waypoints(1, 0) = target_xyz_.y() + data.y;
    xyz_waypoints(2, 0) = target_xyz_.z() + data.z;

    // Replan
    updateCartesianWaypoints(xyz_waypoints);
  }

  bool setCompliantMode(std_srvs::SetBool::Request &req,
		        std_srvs::SetBool::Response &res) {
    if (req.data) {
      // Go into a passive mode so the system can be moved by hand
      res.message = "Pausing active command (entering grav comp mode)";
      arm_.cancelGoal();
      res.success = true;
    } else {
      res.message = "Resuming active command";
      auto t = ::ros::Time::now().toSec();
      auto last_position = arm_.lastFeedback().getPosition();
      arm_.setGoal(arm::Goal::createFromPosition(last_position));
      target_xyz_ = arm_.FK(last_position);
      res.success = true;
    }
    return true;
  }

  void setColor(const Color& color) {
    auto& command = arm_.pendingCommand();
    for (int i = 0; i < command.size(); ++i) {
      command[i].led().set(color);
    }
  }

  void startArmMotion(const hebi_cpp_api_examples::ArmMotionGoalConstPtr& goal) {
    auto last_position = arm_.lastFeedback().getPosition();
    ROS_INFO("Executing arm motion action");

    // Replan a smooth joint trajectory from the current location through a
    // series of cartesian waypoints.
    // TODO: use a single struct instead of 6 single vectors of the same length;
    // but how do we do hierarchial actions?
    size_t num_waypoints = goal->x.size();

    Eigen::Matrix3Xd xyz_positions(3, num_waypoints);
    Eigen::Matrix3Xd tip_directions(3, num_waypoints);

    // Get joint angles to move to each waypoint
    for (size_t i = 0; i < num_waypoints; ++i) {
      // Special homing values...
      if (goal->x[i] == 100 && goal->y[i] == 100 && goal->z[i] == 100) {
        xyz_positions.col(i) = home_position_;
        tip_directions(0, i) = 1;
        tip_directions(1, i) = 0;
        tip_directions(2, i) = 0;
      }
      else if (goal->x[i] == 101 && goal->y[i] == 101 && goal->z[i] == 101) {
        xyz_positions.col(i) = home_position_;
        tip_directions(0, i) = 0;
        tip_directions(1, i) = 0;
        tip_directions(2, i) = -1;
      }
      else {
        xyz_positions(0, i) = goal->x[i];
        xyz_positions(1, i) = goal->y[i];
        xyz_positions(2, i) = goal->z[i];
        tip_directions(0, i) = goal->tipx[i];
        tip_directions(1, i) = goal->tipy[i];
        tip_directions(2, i) = goal->tipz[i];
      }
    }

    updateCartesianWaypoints(xyz_positions, &tip_directions);

    // Set LEDs to a particular color, or clear them.
    Color color;
    if (goal->set_color)
      color = Color(goal->r, goal->g, goal->b, 255);
    setColor(color);

    // Wait until the action is complete, sending status/feedback along the
    // way.
    ::ros::Rate r(10);

    hebi_cpp_api_examples::ArmMotionFeedback feedback;

    while (true) {
      if (action_server_.isPreemptRequested() || !::ros::ok()) {
        ROS_INFO("Arm motion action was preempted");
        setColor({0,0,0,0});
        // Note -- the `startArmMotion` function will not be called until the
        // action server has been preempted here:
        action_server_.setPreempted();
        return;
      }

      if (!action_server_.isActive() || !::ros::ok()) {
        ROS_INFO("Arm motion was cancelled");
        setColor({0,0,0,0});
        return;
      }

      auto t = ::ros::Time::now().toSec();

      // Publish progress:
      feedback.percent_complete = arm_.goalProgress() * 100.0;
      action_server_.publishFeedback(feedback);
 
      if (arm_.atGoal()) {
        break;
      }

      // Limit feedback rate
      r.sleep(); 
    }
    
    setColor({0,0,0,0});

    // publish when the arm is done with a motion
    ROS_INFO("Completed arm motion action");
    action_server_.setSucceeded();
  }

  void publishState() {
    auto& fdbk = arm_.lastFeedback();

    // how is there not a better way to do this?

    // Need to copy data from VectorXd to vector<double> in ros msg
    auto pos = fdbk.getPosition();
    auto vel = fdbk.getVelocity();
    auto eff = fdbk.getEffort();

    state_msg_.position.resize(pos.size());
    state_msg_.velocity.resize(vel.size());
    state_msg_.effort.resize(eff.size());
    state_msg_.header.stamp = ::ros::Time::now();

    VectorXd::Map(&state_msg_.position[0], pos.size()) = pos;
    VectorXd::Map(&state_msg_.velocity[0], vel.size()) = vel;
    VectorXd::Map(&state_msg_.effort[0], eff.size()) = eff;

    arm_state_pub_.publish(state_msg_);

    // compute arm CoM
    auto& model = arm_.robotModel();
    Eigen::VectorXd masses;
    robot_model::Matrix4dVector frames;
    model.getMasses(masses);
    model.getFK(robot_model::FrameType::CenterOfMass, pos, frames);

    center_of_mass_message_.m = 0.0;
    Eigen::Vector3d weighted_sum_com = Eigen::Vector3d::Zero();
    for(int i = 0; i < model.getFrameCount(robot_model::FrameType::CenterOfMass); ++i) {
      center_of_mass_message_.m += masses(i);
      frames[i] *= masses(i);
      weighted_sum_com(0) += frames[i](0, 3);
      weighted_sum_com(1) += frames[i](1, 3);
      weighted_sum_com(2) += frames[i](2, 3);
    }
    weighted_sum_com /= center_of_mass_message_.m;

    center_of_mass_message_.com.x = weighted_sum_com(0);
    center_of_mass_message_.com.y = weighted_sum_com(1);
    center_of_mass_message_.com.z = weighted_sum_com(2);

    center_of_mass_publisher_.publish(center_of_mass_message_);
  }
  
private:
  arm::Arm& arm_;

  // The end effector location that this arm will target (NaN indicates
  // unitialized state, and will be set from feedback during first
  // command)
  Eigen::Vector3d target_xyz_{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  Eigen::VectorXd home_position_;

  Eigen::VectorXd ik_seed_{0};
  bool use_ik_seed_{false};

  ::ros::NodeHandle nh_;

  sensor_msgs::JointState state_msg_;
  geometry_msgs::Inertia center_of_mass_message_;

  actionlib::SimpleActionServer<hebi_cpp_api_examples::ArmMotionAction> action_server_;

  ::ros::Subscriber offset_target_subscriber_;
  ::ros::Subscriber set_target_subscriber_;
  ::ros::Subscriber cartesian_waypoint_subscriber_;
  ::ros::Subscriber joint_waypoint_subscriber_;

  ::ros::Publisher arm_state_pub_;
  ::ros::Publisher center_of_mass_publisher_;

  ::ros::ServiceServer compliant_mode_service_;
  ::ros::ServiceServer ik_seed_service_;

  bool isTargetInitialized() {
    return !std::isnan(target_xyz_.x()) ||
           !std::isnan(target_xyz_.y()) ||
           !std::isnan(target_xyz_.z());
  }

  // Each row is a separate joint; each column is a separate waypoint.
  void updateJointWaypoints(const Eigen::MatrixXd& angles, const Eigen::MatrixXd& velocities, const Eigen::MatrixXd& accelerations, const Eigen::VectorXd& times) {
    // Data sanity check:
    if (angles.rows() != velocities.rows()       || // Number of joints
        angles.rows() != accelerations.rows()    ||
        angles.rows() != arm_.size() ||
        angles.cols() != velocities.cols()       || // Number of waypoints
        angles.cols() != accelerations.cols()    ||
        angles.cols() != times.size()            ||
        angles.cols() == 0) {
      ROS_ERROR("Angles, velocities, accelerations, or times were not the correct size");
      return;
    }

    // Update stored target position, based on final joint angles.
    target_xyz_ = arm_.FK(angles.rightCols<1>());

    // Replan:
    arm_.setGoal(arm::Goal::createFromWaypoints(times, angles, velocities, accelerations));
  }

  // Helper function to condense functionality between various message/action callbacks above
  // Replan a smooth joint trajectory from the current location through a
  // series of cartesian waypoints.
  // xyz positions should be a 3xn vector of target positions
  void updateCartesianWaypoints(const Eigen::Matrix3Xd& xyz_positions,
      const Eigen::Matrix3Xd* end_tip_directions = nullptr) {
    // Data sanity check:
    if (end_tip_directions && end_tip_directions->cols() != xyz_positions.cols())
      return;

    // Update stored target position:
    target_xyz_ = xyz_positions.col(xyz_positions.cols() - 1);

    // These are the joint angles that will be added
    auto num_waypoints = xyz_positions.cols();
    Eigen::MatrixXd positions(arm_.size(), num_waypoints);

    // Plan to each subsequent point from the last position
    // (We use the last position command for smoother motion)
    Eigen::VectorXd last_position = arm_.lastFeedback().getPositionCommand();

    if(use_ik_seed_) {
      last_position = ik_seed_;
    }

    // For each waypoint, find the joint angles to move to it, starting from the last
    // waypoint, and save into the position vector.
    if (end_tip_directions) {
      // If we are given tip directions, add these too...
      for (size_t i = 0; i < num_waypoints; ++i) {
        last_position = arm_.solveIK(last_position, xyz_positions.col(i), static_cast<Eigen::Vector3d>(end_tip_directions->col(i)));

        auto fk_check = arm_.FK(last_position);
        auto mag_diff = (last_position - fk_check).norm();
        if (mag_diff > 0.01) {
          ROS_WARN_STREAM("Target Pose:" << xyz_positions.col(i)[0] << ", "
                                         << xyz_positions.col(i)[1] << ", "
                                         << xyz_positions.col(i)[2]);
          ROS_INFO_STREAM("IK Solution: " << last_position[0] << " | "
                                          << last_position[1] << " | "
                                          << last_position[2] << " | "
                                          << last_position[3] << " | "
                                          << last_position[4] << " | "
                                          << last_position[5]);
          ROS_WARN_STREAM("Pose of IK Solution:" << fk_check[0] << ", "
                                                 << fk_check[1] << ", "
                                                 << fk_check[2]);
          ROS_INFO_STREAM("Distance between target and IK pose: " << mag_diff);
        }
        positions.col(i) = last_position; 
      }
    } else {
      for (size_t i = 0; i < num_waypoints; ++i) {
        last_position = arm_.solveIK(last_position, xyz_positions.col(i));
        positions.col(i) = last_position; 
      }
    }

    // Replan:
    arm_.setGoal(arm::Goal::createFromPositions(positions));
  }
 
};

} // namespace ros
} // namespace hebi

template <typename T>
bool loadParam(ros::NodeHandle node, std::string varname, T& var) {
  if (node.hasParam(varname) && node.getParam(varname, var)) {
    ROS_INFO_STREAM("Found and successfully read '" << varname << "' parameter");
    return true;
  }

  ROS_ERROR_STREAM("Could not find/read required '" << varname << "' parameter!");
  return false;
}

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "arm_node");
  ros::NodeHandle node;

  /////////////////// Load parameters ///////////////////

  // Get parameters for name/family of modules; default to standard values:
  std::vector<std::string> families;
  if (node.hasParam("families") && node.getParam("families", families)) {
    ROS_INFO("Found and successfully read 'families' parameter");
  } else {
    ROS_WARN("Could not find/read 'families' parameter; defaulting to 'HEBI'");
    families = {"HEBI"};
  }

  std::vector<std::string> names;
  // Read the package + path for the gains file
  std::string gains_package;
  std::string gains_file;
  // Read the package + path for the hrdf file
  std::string hrdf_package;
  std::string hrdf_file;

  bool success = true;
  success = success && loadParam(node, "names", names);
  success = success && loadParam(node, "gains_package", gains_package);
  success = success && loadParam(node, "gains_file", gains_file);
  success = success && loadParam(node, "hrdf_package", hrdf_package);
  success = success && loadParam(node, "hrdf_file", hrdf_file);

  if(!success) {
    ROS_ERROR("Could not find one or more required parameters; aborting!");
    return -1;
  }

  // Get the "home" position for the arm
  std::vector<double> home_position_vector;
  if (node.hasParam("home_position") && node.getParam("home_position", home_position_vector)) {
    ROS_INFO("Found and successfully read 'home_position' parameter");
  } else {
    ROS_WARN("Could not find/read 'home_position' parameter; defaulting to all zeros!");
  }

  /////////////////// Initialize arm ///////////////////

  // Create arm
  arm::Arm::Params params;
  params.families_ = families;
  params.names_ = names;
  params.get_current_time_s_ = []() {
    static double start_time = ros::Time::now().toSec();
    return ros::Time::now().toSec() - start_time;
  };

  params.hrdf_file_ = ros::package::getPath(hrdf_package) + std::string("/") + hrdf_file;

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
    ROS_ERROR_STREAM("Failed to find the following modules in family: " << families.at(0));
    for(auto it = names.begin(); it != names.end(); ++it) {
        ROS_ERROR_STREAM("> " << *it);
    }
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
    return -1;
  } else {
    for (size_t i = 0; i < home_position.size(); ++i) {
      home_position[i] = home_position_vector[i];
    }
  }

  // Make a list of family/actuator formatted names for the JointState publisher
  std::vector<std::string> full_names;
  for (size_t idx=0; idx<names.size(); ++idx) {
    full_names.push_back(families.at(0) + "/" + names.at(idx));
  }

  /////////////////// Initialize ROS interface ///////////////////

  hebi::ros::ArmNode arm_node(&node, *arm, home_position, full_names);

  if (node.hasParam("ik_seed")) {
    std::vector<double> ik_seed;
    node.getParam("ik_seed", ik_seed);
    arm_node.setIKSeed(ik_seed);
  } else {
    ROS_WARN("Param ik_seed not set, arm may exhibit erratic behavior");
  }

  /////////////////// Main Loop ///////////////////

  // We update with a current timestamp so the "setGoal" function
  // is planning from the correct time for a smooth start

  auto t = ros::Time::now();

  arm->update();
  arm->setGoal(arm::Goal::createFromPosition(home_position));

  auto prev_t = t;
  while (ros::ok()) {
    t = ros::Time::now();

    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm->update())
      ROS_WARN("Error Getting Feedback -- Check Connection");
    else if (!arm->send())
      ROS_WARN("Error Sending Commands -- Check Connection");

    arm_node.publishState();

    // If a simulator reset has occured, go back to the home position.
    if (t < prev_t) {
      std::cout << "Returning to home pose after simulation reset" << std::endl;
      arm->setGoal(arm::Goal::createFromPosition(home_position));
    }
    prev_t = t;

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
