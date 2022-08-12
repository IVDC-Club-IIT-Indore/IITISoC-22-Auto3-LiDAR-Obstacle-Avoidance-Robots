#include "odom_publisher.hpp"

#include <ros/ros.h>

namespace hebi {
namespace ros {

OdomPublisher::OdomPublisher(::ros::NodeHandle& node)
  : pub_(node.advertise<nav_msgs::Odometry>("odom", 100)) {
  // Parameter setup for NAV and Geometry messages
  // Odom and tf message setup; stamp and frame id assignment
  msg_.header.frame_id = "odom";
  msg_.child_frame_id = "base_footprint";

  tf_trans_.header.frame_id = "odom";
  tf_trans_.child_frame_id = "base_footprint";
}
  
void OdomPublisher::send(const ::ros::Time& time, const Eigen::Vector3d& global_pose_, const Eigen::Vector3d& global_vel_) {
  // Timestamps
  msg_.header.stamp = time;
  tf_trans_.header.stamp = time;

  // Update messages
  auto quat = tf::createQuaternionMsgFromYaw(global_pose_[2]);

  // Position (odom pub)
  msg_.pose.pose.position.x = global_pose_.x();
  msg_.pose.pose.position.y = global_pose_.y();
  msg_.pose.pose.orientation = quat;

  // Velocity (odom pub)
  msg_.twist.twist.linear.x = global_vel_.x();
  msg_.twist.twist.linear.y = global_vel_.y();
  msg_.twist.twist.angular.z = global_vel_.z();

  // Position (tf)
  tf_trans_.transform.translation.x = global_pose_.x();
  tf_trans_.transform.translation.y = global_pose_.y();
  tf_trans_.transform.rotation = quat;

  // Publish odometry transform and message
  broadcaster_.sendTransform(tf_trans_);
  pub_.publish(msg_);
}

} // namespace ros
} // namespace hebi
