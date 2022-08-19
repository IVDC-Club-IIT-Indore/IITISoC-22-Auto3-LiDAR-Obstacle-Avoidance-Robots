#include "waypoint.hpp"

#include <fstream>
#include <iostream>

namespace hebi {

Waypoint::Waypoint(const std::string& name, const std::vector<double>& waypoint)
  : name_(name) {
  angles_.resize(waypoint.size());
  for (size_t i = 0; i < waypoint.size(); ++i) {
    angles_(i) = waypoint[i];
  }
}

void Waypoint::write(const std::string& filename) {
  std::ofstream file;
  file.open(filename, std::ofstream::out); 
  file << "data/waypoints/" << name_ << ": [\n";
  for (int i = 0; i < angles_.size(); ++i)
    file << angles_(i) << ",";
  file << "\n]";
  file.close();
}

} // namespace hebi
