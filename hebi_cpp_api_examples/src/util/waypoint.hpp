#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace hebi {

// A helper class that stores a joint angle waypoint
class Waypoint {

public:

  Waypoint(const std::string& name, const Eigen::VectorXd& waypoint)
    : name_(name), angles_(waypoint) {}
  Waypoint(const std::string& name, const std::vector<double>& waypoint);

  // Save as a ros-param compatible file.
  void write(const std::string& filename);
  
  // Get the name of this waypoint
  const std::string& name() const { return name_; }
  // Get the joint angles for this waypoint
  const Eigen::VectorXd& angles() const { return angles_; }
  void setAngles(const Eigen::VectorXd& angles) { angles_ = angles; }

private:
  std::string name_;
  Eigen::VectorXd angles_;
};

} // namespace hebi
