#ifndef CTF_NAVIGATION_VISION_LASER_UTILS_HPP
#define CTF_NAVIGATION_VISION_LASER_UTILS_HPP

#include <cmath>
#include <limits>

#include <boost/optional.hpp>
#include <sensor_msgs/LaserScan.h>

namespace ctf_navigation
{
namespace vision
{

/// Rango mínimo válido en una ventana angular alrededor de bearing (rad).
inline boost::optional<double> minRangeAtBearing(const sensor_msgs::LaserScan& scan,
                                                  double bearing,
                                                  double window_rad)
{
  double best = std::numeric_limits<double>::infinity();
  bool found = false;

  const size_t n = scan.ranges.size();
  for (size_t i = 0; i < n; ++i)
  {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double diff = std::atan2(std::sin(angle - bearing), std::cos(angle - bearing));
    if (std::fabs(diff) > window_rad)
    {
      continue;
    }
    const float r = scan.ranges[i];
    if (std::isinf(r) || std::isnan(r) || r < scan.range_min || r > scan.range_max)
    {
      continue;
    }
    if (static_cast<double>(r) < best)
    {
      best = r;
      found = true;
    }
  }

  if (!found)
  {
    return boost::none;
  }
  return best;
}

/// Rango mínimo entre angle_min y angle_max (rad, marco del LaserScan).
inline boost::optional<double> minRangeInAngleRange(const sensor_msgs::LaserScan& scan,
                                                    double angle_min,
                                                    double angle_max)
{
  double best = std::numeric_limits<double>::infinity();
  bool found = false;

  const size_t n = scan.ranges.size();
  for (size_t i = 0; i < n; ++i)
  {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (angle < angle_min || angle > angle_max)
    {
      continue;
    }
    const float r = scan.ranges[i];
    if (std::isinf(r) || std::isnan(r) || r < scan.range_min || r > scan.range_max)
    {
      continue;
    }
    if (static_cast<double>(r) < best)
    {
      best = r;
      found = true;
    }
  }

  if (!found)
  {
    return boost::none;
  }
  return best;
}

}  // namespace vision
}  // namespace ctf_navigation

#endif
