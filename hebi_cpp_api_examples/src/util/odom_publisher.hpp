#pragma once

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>

namespace hebi {
namespace ros {

class OdomPublisher {
public:
  OdomPublisher(::ros::NodeHandle& node);

  void send(const ::ros::Time& time, const Eigen::Vector3d& global_pose_, const Eigen::Vector3d& global_vel_);

private:
  // Posts the calculated odometry to a topic
  ::ros::Publisher pub_;
  nav_msgs::Odometry msg_;

  //Create a broadcaster for the TF for odom -> base_link
  tf::TransformBroadcaster broadcaster_;
  geometry_msgs::TransformStamped tf_trans_;
};

} // namespace ros
} // namespace hebi
