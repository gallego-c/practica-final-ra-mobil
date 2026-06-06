#ifndef CTF_NAVIGATION_COMMON_GEOMETRY_HPP
#define CTF_NAVIGATION_COMMON_GEOMETRY_HPP

#include <cmath>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ctf_navigation
{
namespace geometry
{

inline double distance2D(double x0, double y0, double x1, double y1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  return std::sqrt(dx * dx + dy * dy);
}

inline double distanceBetween(const geometry_msgs::TransformStamped& a,
                              const geometry_msgs::TransformStamped& b)
{
  return distance2D(a.transform.translation.x, a.transform.translation.y,
                  b.transform.translation.x, b.transform.translation.y);
}

inline double normalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

inline geometry_msgs::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

inline double yawFromTransform(const geometry_msgs::TransformStamped& t)
{
  const auto& q = t.transform.rotation;
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

}  // namespace geometry
}  // namespace ctf_navigation

#endif
