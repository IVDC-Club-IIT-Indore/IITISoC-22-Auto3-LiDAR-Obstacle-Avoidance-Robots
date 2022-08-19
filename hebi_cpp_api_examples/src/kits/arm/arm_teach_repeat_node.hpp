#include <Eigen/Dense>

#include <std_msgs/Bool.h>

#include "hebi_cpp_api_examples/SaveWaypoint.h"
#include "hebi_cpp_api_examples/Playback.h"
#include "hebi_cpp_api_examples/StartPath.h"
#include "hebi_cpp_api_examples/EndPath.h"
#include "hebi_cpp_api_examples/OffsetPlayback.h"

#include "hebi_cpp_api/arm/arm.hpp"

#include "src/util/waypoint.hpp"
#include "src/util/path.hpp"

namespace arm = hebi::experimental::arm;

namespace hebi {
namespace ros {

// A class that can be used as the basis of a ROS node for a teach/repeat demonstration.
// This class supports saving/playback of paths and waypoint, and provides callback-able
// functions that use ROS messages.
//
// The "update" function should be called in a loop on the underlying "arm" and this class
// itself.
class TeachRepeatNode {
public:
  TeachRepeatNode(arm::Arm& arm) : arm_(arm) { }

  ////////////////////////////////////////////////////////
  // Callback functions for ROS subscribers:
  
  // Turn on or off a "compliant" or "passive" mode so the system can be moved by hand
  void setCompliantMode(std_msgs::Bool msg);

  // Offset the desired next playback position
  //
  // Also jog the current end effector location in (x,y,z) space, replanning
  // smoothly to the new location
  //
  // Note -- this is cleared after playing through a path!
  void offsetWaypointPlayback(hebi_cpp_api_examples::OffsetPlayback msg);

  // Save the current position as a new waypoint.
  void saveWaypoint(hebi_cpp_api_examples::SaveWaypoint msg);

  // Start/end recording a path
  void startRecordPath(hebi_cpp_api_examples::StartPath msg);
  void endRecordPath(hebi_cpp_api_examples::EndPath msg);
 
  // Playback, either to a waypoint or through a path
  void startPlayback(hebi_cpp_api_examples::Playback msg);

  ////////////////////////////////////////////////////////

  // Updates any actively-being-constructed paths.
  void update(double t);

  // Add path (e.g., during initialization)
  void addPath(const Path& path) { paths_.push_back(path); }

  // Add waypoint (e.g., during initialization)
  void addWaypoint(const Waypoint& wp) { waypoints_.push_back(wp); }

  // Needed when member variables are fixed-size Eigen structures
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  hebi::experimental::arm::Arm& arm_;

  // The end effector location that this arm will nominally target (NaN indicates
  // unitialized state, and will be set from feedback during first
  // command)
  Eigen::Vector3d target_xyz_{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  // Allow offsetting waypoints by a given "xyz" amount during playback.
  Eigen::Vector3d waypoint_playback_offset_{0, 0, 0};

  std::vector<Waypoint> waypoints_;
  std::vector<Path> paths_;

  std::vector<Eigen::VectorXd> currently_constructing_path_;
  bool constructing_path_{};
  double last_path_point_time_;

  // How frequently we save waypoints while recording a path (in seconds).
  const double path_dt_ { 0.25 };

  // Helper function that indicates whether or not was have received a
  // command yet.
  bool isTargetInitialized() {
    return !(std::isnan(target_xyz_.x()) || std::isnan(target_xyz_.y()) || std::isnan(target_xyz_.z()));
  }

  // Shifts the output of the robot by (xyz) while keeping the orientation the same;
  // modifies "joint_angles" in place
  Eigen::VectorXd offsetJoints(const Eigen::VectorXd& joint_angles, const Eigen::Vector3d& xyz_offset) const;

  void playPath(size_t path_index);
  void goToWaypoint(const Eigen::VectorXd& joint_angles);
};

} // namespace hebi
} // namespace ros
