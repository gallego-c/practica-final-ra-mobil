#ifndef CTF_NAVIGATION_GAME_ROBOT_AGENT_HPP
#define CTF_NAVIGATION_GAME_ROBOT_AGENT_HPP

#include <boost/optional.hpp>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/buffer.h>

#include "ctf_navigation/common/move_base_client.hpp"
#include "ctf_navigation/common/tf_helper.hpp"
#include "ctf_navigation/game/types.hpp"

namespace ctf_navigation
{
namespace game
{

enum class SearchState
{
  EXPLORING,
  PURSUING_FLAG
};

/// Un robot en el juego: move_base, TF, y estado de detección de bandera.
class RobotAgent
{
public:
  RobotAgent(const RobotAgent&) = delete;
  RobotAgent& operator=(const RobotAgent&) = delete;
  RobotAgent(RobotAgent&&) = delete;
  RobotAgent& operator=(RobotAgent&&) = delete;

  RobotAgent(const std::string& ns,
             const std::string& base_frame,
             const HomePose& home,
             const std::string& map_frame,
             const std::string& detector_node,
             tf2_ros::Buffer& tf_buffer);

  bool waitForMoveBase(double timeout_sec = 60.0);

  boost::optional<tf_helper::Pose2D> poseInMap() const;

  void sendGoal(double x, double y, double yaw);
  void cancelGoals();

  bool moveBaseSucceeded() const;
  bool moveBaseTerminal() const;

  // Visión
  bool flagSeen() const { return flag_seen_; }
  bool hasFreshFlagEstimate(double timeout_sec) const;
  boost::optional<std::pair<double, double>> flagEstimate() const;

  SearchState searchState() const { return search_state_; }
  void setSearchState(SearchState s) { search_state_ = s; }

  int targetWaypoint() const { return target_waypoint_; }
  void setTargetWaypoint(int idx) { target_waypoint_ = idx; }
  void clearTargetWaypoint() { target_waypoint_ = -1; }

  void resetFlagGoalThrottle();
  void resetForPhaseTransition();
  bool shouldSendFlagGoal(double fx, double fy, double min_move_m, double max_age_sec);

  const std::string& ns() const { return ns_; }
  const HomePose& home() const { return home_; }

private:
  void onFlagFound(const std_msgs::Bool::ConstPtr& msg);
  void onFlagEstimate(const geometry_msgs::PoseStamped::ConstPtr& msg);

  std::string ns_;
  std::string base_frame_;
  std::string map_frame_;
  HomePose home_;

  MoveBaseClientWrapper move_base_;
  tf2_ros::Buffer& tf_buffer_;

  SearchState search_state_ = SearchState::EXPLORING;
  int target_waypoint_ = -1;

  bool flag_seen_ = false;
  boost::optional<std::pair<double, double>> flag_estimate_;
  ros::Time flag_estimate_time_;

  boost::optional<std::pair<double, double>> last_flag_goal_;
  ros::Time last_flag_goal_time_;

  // Deben ser miembros: si son temporales en el ctor, la suscripción se cancela.
  ros::Subscriber flag_found_sub_;
  ros::Subscriber flag_estimate_sub_;
};

}  // namespace game
}  // namespace ctf_navigation

#endif
