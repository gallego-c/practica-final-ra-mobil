#ifndef CTF_NAVIGATION_GAME_EXPLORATION_HPP
#define CTF_NAVIGATION_GAME_EXPLORATION_HPP

#include <set>
#include <utility>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>

namespace ctf_navigation
{
namespace game
{

using Waypoint = std::pair<double, double>;

/// Genera waypoints de cobertura en celdas libres del mapa de ocupación.
class ExplorationPlanner
{
public:
  void configure(double step_m, double clearance_m);

  /// Builds the waypoint grid once, preserving the original static-map behavior.
  void buildFromMap(const nav_msgs::OccupancyGrid& grid);

  /// Rebuilds waypoints from the latest map. In SLAM maps, frontier-like
  /// waypoints near unknown cells are preferred; fully known maps fall back to
  /// coverage waypoints.
  void updateFromMap(const nav_msgs::OccupancyGrid& grid);

  bool ready() const { return !waypoints_.empty(); }
  const std::vector<Waypoint>& waypoints() const { return waypoints_; }

  /// Índice del waypoint libre más cercano a (x,y), excluyendo exclude_index.
  int nearestUnvisited(double x, double y, int exclude_index) const;

  void markVisited(int index);
  void resetVisitedIfComplete();

  bool isVisited(int index) const;
  int visitedCount() const { return static_cast<int>(visited_.size()); }

private:
  double step_m_ = 1.0;
  double clearance_m_ = 0.30;
  std::vector<Waypoint> waypoints_;
  std::set<int> visited_;
};

}  // namespace game
}  // namespace ctf_navigation

#endif
