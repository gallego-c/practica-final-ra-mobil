#ifndef CTF_NAVIGATION_GAME_MAP_UTILS_HPP
#define CTF_NAVIGATION_GAME_MAP_UTILS_HPP

#include <nav_msgs/OccupancyGrid.h>

namespace ctf_navigation
{
namespace game
{

/// Snap (wx, wy) to the nearest collision-free cell on the occupancy grid.
/// Returns true if a valid cell was found (coordinates updated in place).
bool snapGoalToFree(const nav_msgs::OccupancyGrid& grid,
                    double& wx,
                    double& wy,
                    double clearance_m,
                    double max_search_m = 2.0);

}  // namespace game
}  // namespace ctf_navigation

#endif
