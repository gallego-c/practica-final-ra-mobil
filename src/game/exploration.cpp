#include "ctf_navigation/game/exploration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "ctf_navigation/common/geometry.hpp"

namespace ctf_navigation
{
namespace game
{

void ExplorationPlanner::configure(double step_m, double clearance_m)
{
  step_m_ = step_m;
  clearance_m_ = clearance_m;
}

void ExplorationPlanner::buildFromMap(const nav_msgs::OccupancyGrid& grid)
{
  if (!waypoints_.empty())
  {
    return;
  }

  const double res = grid.info.resolution;
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);

  const int step_cells = std::max(1, static_cast<int>(std::round(step_m_ / res)));
  const int clear_cells = std::max(1, static_cast<int>(std::round(clearance_m_ / res)));

  auto occupancy = [&](int cx, int cy) -> int {
    if (cx < 0 || cy < 0 || cx >= w || cy >= h)
    {
      return 100;
    }
    return grid.data[cy * w + cx];
  };

  auto isClear = [&](int cx, int cy) -> bool {
    for (int dy = -clear_cells; dy <= clear_cells; ++dy)
    {
      for (int dx = -clear_cells; dx <= clear_cells; ++dx)
      {
        const int v = occupancy(cx + dx, cy + dy);
        if (v < 0 || v >= 50)
        {
          return false;
        }
      }
    }
    return true;
  };

  for (int cy = clear_cells; cy < h - clear_cells; cy += step_cells)
  {
    for (int cx = clear_cells; cx < w - clear_cells; cx += step_cells)
    {
      if (isClear(cx, cy))
      {
        const double wx = ox + (cx + 0.5) * res;
        const double wy = oy + (cy + 0.5) * res;
        waypoints_.emplace_back(wx, wy);
      }
    }
  }
}

int ExplorationPlanner::nearestUnvisited(double x, double y, int exclude_index) const
{
  int best = -1;
  double best_d = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < waypoints_.size(); ++i)
  {
    if (visited_.count(static_cast<int>(i)) > 0 ||
        static_cast<int>(i) == exclude_index)
    {
      continue;
    }
    const double d = geometry::distance2D(x, y, waypoints_[i].first, waypoints_[i].second);
    if (d < best_d)
    {
      best_d = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

void ExplorationPlanner::markVisited(int index)
{
  if (index >= 0 && static_cast<size_t>(index) < waypoints_.size())
  {
    visited_.insert(index);
  }
}

void ExplorationPlanner::resetVisitedIfComplete()
{
  if (!waypoints_.empty() &&
      visited_.size() >= waypoints_.size())
  {
    visited_.clear();
  }
}

bool ExplorationPlanner::isVisited(int index) const
{
  if (index < 0 || static_cast<size_t>(index) >= waypoints_.size())
  {
    return false;
  }
  return visited_.count(index) > 0;
}

}  // namespace game
}  // namespace ctf_navigation
