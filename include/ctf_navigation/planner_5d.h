#ifndef CTF_NAVIGATION_PLANNER_5D_H
#define CTF_NAVIGATION_PLANNER_5D_H

#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

namespace ctf_navigation
{

struct State5D
{
  double x;
  double y;
  double theta;
  double v;
  double omega;
};

/// Planificador local por rollout de velocidades (v, omega) sobre el costmap.
class Planner5D : public nav_core::BaseLocalPlanner
{
public:
  Planner5D();

  void initialize(std::string name,
                  tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

private:
  State5D simulate(const State5D& s, double v, double omega, double dt) const;
  bool isCollision(const State5D& s) const;
  State5D getCurrentState() const;

  double max_vel_x_;
  double min_vel_x_;
  double max_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double sim_time_;
  double sim_step_;
  int v_samples_;
  int omega_samples_;
  int max_iterations_;

  bool initialized_;
  bool goal_reached_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped goal_pose_;

  double path_follow_weight_;
  double proximity_weight_;

  int escape_mode_;
  int rotation_sign_;
  ros::Time last_escape_switch_;

  costmap_2d::Costmap2DROS* costmap_ros_;
  tf2_ros::Buffer* tf_;
};

}  // namespace ctf_navigation

#endif
