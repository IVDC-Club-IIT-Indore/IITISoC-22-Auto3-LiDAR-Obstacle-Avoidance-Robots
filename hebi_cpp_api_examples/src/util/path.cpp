#include "path.hpp"

#include <fstream>
#include <iostream>

namespace hebi {

Path::Path(const std::string& name, const std::vector<double>& points, size_t num_joints) 
  : name_(name) {
  auto num_pts = points.size() / num_joints;
  Eigen::VectorXd new_pt(num_joints);
  for (size_t pt = 0; pt < num_pts; ++pt) {
    for (size_t j = 0; j < num_joints; ++j) {
      new_pt(j) = points[num_joints * pt + j];
    }
    waypoints_.push_back(new_pt);
  }
}

void Path::write(const std::string& filename) {
  std::ofstream file;
  file.open(filename, std::ofstream::out); 
  file << "data/paths/" << name_ << ": [\n";
  for (const auto& wp : waypoints_) {
    for (int i = 0; i < wp.size(); ++i)
      file << wp(i) << ",";
    file << "\n"; // Make it readable by putting each waypoint on a separate line...
  }
  file << "\n]";
  file.close();
}

} // namespace hebi
