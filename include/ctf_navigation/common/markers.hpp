#ifndef CTF_NAVIGATION_COMMON_MARKERS_HPP
#define CTF_NAVIGATION_COMMON_MARKERS_HPP

#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ctf_navigation
{
namespace markers
{

inline visualization_msgs::Marker makeCylinder(const std::string& frame,
                                               const std::string& ns,
                                               int id,
                                               double x,
                                               double y,
                                               double radius,
                                               double height,
                                               float r,
                                               float g,
                                               float b,
                                               float a)
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::CYLINDER;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = height * 0.5;
  m.pose.orientation.w = 1.0;
  m.scale.x = radius;
  m.scale.y = radius;
  m.scale.z = height;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = ros::Duration(0.0);
  return m;
}

inline visualization_msgs::Marker makeSphere(const std::string& frame,
                                               const std::string& ns,
                                               int id,
                                               double x,
                                               double y,
                                               double z,
                                               double scale,
                                               float r,
                                               float g,
                                               float b,
                                               float a)
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = z;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = ros::Duration(0.0);
  return m;
}

}  // namespace markers
}  // namespace ctf_navigation

#endif
