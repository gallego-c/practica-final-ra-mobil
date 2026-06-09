#include "ctf_navigation/game/map_utils.hpp"

#include <algorithm>
#include <cmath>

namespace ctf_navigation
{
namespace game
{

namespace
{

constexpr int kFreeThresh = 50;

int occupancyAt(const nav_msgs::OccupancyGrid& grid, int cx, int cy)
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  if (cx < 0 || cy < 0 || cx >= w || cy >= h)
  {
    return 100;
  }
  return grid.data[cy * w + cx];
}

bool cellHasClearance(const nav_msgs::OccupancyGrid& grid, int cx, int cy, int clear_cells)
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  if (cx < clear_cells || cy < clear_cells || cx >= w - clear_cells || cy >= h - clear_cells)
  {
    return false;
  }

  const int center = occupancyAt(grid, cx, cy);
  if (center < 0 || center >= kFreeThresh)
  {
    return false;
  }

  for (int dy = -clear_cells; dy <= clear_cells; ++dy)
  {
    for (int dx = -clear_cells; dx <= clear_cells; ++dx)
    {
      const int v = occupancyAt(grid, cx + dx, cy + dy);
      // Occupied cells block the goal; unknown cells are allowed (SLAM maps).
      if (v >= kFreeThresh)
      {
        return false;
      }
    }
  }
  return true;
}

void worldToMap(const nav_msgs::OccupancyGrid& grid, double wx, double wy, int& mx, int& my)
{
  const double res = grid.info.resolution;
  mx = static_cast<int>((wx - grid.info.origin.position.x) / res);
  my = static_cast<int>((wy - grid.info.origin.position.y) / res);
}

void mapToWorld(const nav_msgs::OccupancyGrid& grid, int mx, int my, double& wx, double& wy)
{
  const double res = grid.info.resolution;
  wx = grid.info.origin.position.x + (mx + 0.5) * res;
  wy = grid.info.origin.position.y + (my + 0.5) * res;
}

}  // namespace

bool snapGoalToFree(const nav_msgs::OccupancyGrid& grid,
                    double& wx,
                    double& wy,
                    double clearance_m,
                    double max_search_m)
{
  if (grid.data.empty() || grid.info.width == 0 || grid.info.height == 0)
  {
    return false;
  }

  const double res = grid.info.resolution;
  const int clear_cells = std::max(1, static_cast<int>(std::round(clearance_m / res)));
  const int max_radius = std::max(1, static_cast<int>(std::ceil(max_search_m / res)));

  int mx = 0;
  int my = 0;
  worldToMap(grid, wx, wy, mx, my);

  if (cellHasClearance(grid, mx, my, clear_cells))
  {
    mapToWorld(grid, mx, my, wx, wy);
    return true;
  }

  double best_dist_sq = -1.0;
  double best_wx = wx;
  double best_wy = wy;
  bool found = false;

  for (int radius = 1; radius <= max_radius; ++radius)
  {
    for (int dy = -radius; dy <= radius; ++dy)
    {
      for (int dx = -radius; dx <= radius; ++dx)
      {
        if (std::abs(dx) != radius && std::abs(dy) != radius)
        {
          continue;
        }
        const int cx = mx + dx;
        const int cy = my + dy;
        if (!cellHasClearance(grid, cx, cy, clear_cells))
        {
          continue;
        }
        double candidate_wx = 0.0;
        double candidate_wy = 0.0;
        mapToWorld(grid, cx, cy, candidate_wx, candidate_wy);
        const double dist_sq =
            (candidate_wx - wx) * (candidate_wx - wx) + (candidate_wy - wy) * (candidate_wy - wy);
        if (!found || dist_sq < best_dist_sq)
        {
          best_dist_sq = dist_sq;
          best_wx = candidate_wx;
          best_wy = candidate_wy;
          found = true;
        }
      }
    }
    if (found)
    {
      wx = best_wx;
      wy = best_wy;
      return true;
    }
  }

  return false;
}

}  // namespace game
}  // namespace ctf_navigation
