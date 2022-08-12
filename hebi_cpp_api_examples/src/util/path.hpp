#pragma once

#include <vector>
#include <string>

#include <Eigen/Dense>

namespace hebi {

// A helper class that stores a path as a series of joint angle waypoints
class Path {
public:
  using WaypointList = std::vector<Eigen::VectorXd>;

  Path(const std::string& name, const WaypointList& waypoints)
    : name_(name), waypoints_(waypoints) {}
  Path(const std::string& name, const std::vector<double>& points, size_t num_joints);

  // Save as a ros-param compatible file.
  void write(const std::string& filename);

  // Get the name of this path
  const std::string& name() const { return name_; }
  // Return the number of waypoints in this path
  const size_t size() const { return waypoints_.size(); }
  // Get the `i`th waypoint
  const Eigen::VectorXd& waypoint(size_t i) const { return waypoints_[i]; }

  void setPath(const WaypointList& waypoints) { waypoints_ = waypoints; }

private:
  std::string name_;
  WaypointList waypoints_;
};

} // namespace hebi
