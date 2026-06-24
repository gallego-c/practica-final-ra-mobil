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
  updateFromMap(grid);
}

void ExplorationPlanner::updateFromMap(const nav_msgs::OccupancyGrid& grid)
{
  if (grid.info.resolution <= 0.0 || grid.info.width == 0 || grid.info.height == 0 ||
      grid.data.empty())
  {
    return;
  }

  std::vector<Waypoint> old_visited;
  for (const int index : visited_)
  {
    if (index >= 0 && static_cast<size_t>(index) < waypoints_.size())
    {
      old_visited.push_back(waypoints_[static_cast<size_t>(index)]);
    }
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

  auto isUnknown = [&](int cx, int cy) -> bool {
    if (cx < 0 || cy < 0 || cx >= w || cy >= h)
    {
      return false;
    }
    return grid.data[cy * w + cx] < 0;
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

  const int frontier_window_cells = std::max(clear_cells + 1, step_cells / 2);
  auto isNearUnknown = [&](int cx, int cy) -> bool {
    for (int dy = -frontier_window_cells; dy <= frontier_window_cells; ++dy)
    {
      for (int dx = -frontier_window_cells; dx <= frontier_window_cells; ++dx)
      {
        if (isUnknown(cx + dx, cy + dy))
        {
          return true;
        }
      }
    }
    return false;
  };

  std::vector<Waypoint> coverage_waypoints;
  std::vector<Waypoint> frontier_waypoints;

  for (int cy = clear_cells; cy < h - clear_cells; cy += step_cells)
  {
    for (int cx = clear_cells; cx < w - clear_cells; cx += step_cells)
    {
      if (isClear(cx, cy))
      {
        const double wx = ox + (cx + 0.5) * res;
        const double wy = oy + (cy + 0.5) * res;
        coverage_waypoints.emplace_back(wx, wy);
        if (isNearUnknown(cx, cy))
        {
          frontier_waypoints.emplace_back(wx, wy);
        }
      }
    }
  }

  waypoints_ = frontier_waypoints.empty() ? coverage_waypoints : frontier_waypoints;
  visited_.clear();

  const double visited_match_dist = std::max(0.5 * step_m_, clearance_m_);
  for (size_t i = 0; i < waypoints_.size(); ++i)
  {
    for (const auto& visited_wp : old_visited)
    {
      if (geometry::distance2D(waypoints_[i].first, waypoints_[i].second,
                               visited_wp.first, visited_wp.second) <=
          visited_match_dist)
      {
        visited_.insert(static_cast<int>(i));
        break;
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
