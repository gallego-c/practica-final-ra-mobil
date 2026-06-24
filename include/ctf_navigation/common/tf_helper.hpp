#ifndef CTF_NAVIGATION_COMMON_TF_HELPER_HPP
#define CTF_NAVIGATION_COMMON_TF_HELPER_HPP

#include <boost/optional.hpp>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include "ctf_navigation/common/geometry.hpp"

namespace ctf_navigation
{
namespace tf_helper
{

inline bool lookupTransform(tf2_ros::Buffer& buffer,
                            const std::string& target_frame,
                            const std::string& source_frame,
                            geometry_msgs::TransformStamped& out,
                            double timeout_sec = 0.2)
{
  try
  {
    out = buffer.lookupTransform(target_frame, source_frame, ros::Time(0),
                                 ros::Duration(timeout_sec));
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "TF %s -> %s: %s", source_frame.c_str(),
                      target_frame.c_str(), ex.what());
    return false;
  }
}

struct Pose2D
{
  double x;
  double y;
  double yaw;
};

inline boost::optional<Pose2D> lookupPose2D(tf2_ros::Buffer& buffer,
                                              const std::string& map_frame,
                                              const std::string& base_frame)
{
  geometry_msgs::TransformStamped tf;
  if (!lookupTransform(buffer, map_frame, base_frame, tf))
  {
    return boost::none;
  }
  Pose2D pose;
  pose.x = tf.transform.translation.x;
  pose.y = tf.transform.translation.y;
  pose.yaw = geometry::yawFromTransform(tf);
  return pose;
}

inline boost::optional<geometry_msgs::PoseStamped> transformPose(
    tf2_ros::Buffer& buffer,
    const geometry_msgs::PoseStamped& pose_in,
    const std::string& target_frame,
    double timeout_sec = 0.2)
{
  try
  {
    // Multi-PC real robots: never use the sensor header stamp for TF lookup.
    // Camera/scan stamps often come from a different clock than the roslaunch PC.
    geometry_msgs::PoseStamped pose_latest = pose_in;
    pose_latest.header.stamp = ros::Time(0);
    geometry_msgs::PoseStamped out;
    out = buffer.transform(pose_latest, target_frame, ros::Duration(timeout_sec));
    return out;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "TF transform to %s failed: %s",
                      target_frame.c_str(), ex.what());
    return boost::none;
  }
}

}  // namespace tf_helper
}  // namespace ctf_navigation

#endif
