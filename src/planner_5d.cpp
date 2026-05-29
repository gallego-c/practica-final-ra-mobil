#include <ctf_navigation/planner_5d.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace
{

double normalizeAngle(double angle)
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

double distance2D(double x0, double y0, double x1, double y1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

namespace ctf_navigation
{

Planner5D::Planner5D()
  : max_vel_x_(0.26)
  , min_vel_x_(0.0)
  , max_vel_theta_(1.82)
  , acc_lim_x_(2.5)
  , acc_lim_theta_(3.2)
  , xy_goal_tolerance_(0.15)
  , yaw_goal_tolerance_(0.1)
  , sim_time_(1.5)
  , sim_step_(0.08)
  , v_samples_(12)
  , omega_samples_(24)
  , max_iterations_(500)
  , initialized_(false)
  , goal_reached_(false)
  , rotation_sign_(1)
  , last_rotation_switch_(0)
  , costmap_ros_(nullptr)
  , tf_(nullptr)
{
}

void Planner5D::initialize(std::string name,
                           tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("Planner5D already initialized");
    return;
  }

  ros::NodeHandle private_nh("~/" + name);
  ros::NodeHandle fallback_nh("~/Planner5D");
  private_nh.param("max_vel_x", max_vel_x_, max_vel_x_);
  private_nh.param("min_vel_x", min_vel_x_, min_vel_x_);
  private_nh.param("max_vel_theta", max_vel_theta_, max_vel_theta_);
  private_nh.param("acc_lim_x", acc_lim_x_, acc_lim_x_);
  private_nh.param("acc_lim_theta", acc_lim_theta_, acc_lim_theta_);
  private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, xy_goal_tolerance_);
  private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, yaw_goal_tolerance_);
  private_nh.param("sim_time", sim_time_, sim_time_);
  private_nh.param("sim_step", sim_step_, sim_step_);
  private_nh.param("v_samples", v_samples_, v_samples_);
  private_nh.param("omega_samples", omega_samples_, omega_samples_);
  private_nh.param("max_iterations", max_iterations_, max_iterations_);
  fallback_nh.param("max_vel_x", max_vel_x_, max_vel_x_);
  fallback_nh.param("min_vel_x", min_vel_x_, min_vel_x_);
  fallback_nh.param("max_vel_theta", max_vel_theta_, max_vel_theta_);
  fallback_nh.param("acc_lim_x", acc_lim_x_, acc_lim_x_);
  fallback_nh.param("acc_lim_theta", acc_lim_theta_, acc_lim_theta_);
  fallback_nh.param("xy_goal_tolerance", xy_goal_tolerance_, xy_goal_tolerance_);
  fallback_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, yaw_goal_tolerance_);
  fallback_nh.param("sim_time", sim_time_, sim_time_);
  fallback_nh.param("sim_step", sim_step_, sim_step_);
  fallback_nh.param("v_samples", v_samples_, v_samples_);
  fallback_nh.param("omega_samples", omega_samples_, omega_samples_);
  fallback_nh.param("max_iterations", max_iterations_, max_iterations_);

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  initialized_ = true;
  goal_reached_ = false;

  ROS_INFO("Planner5D initialized in frame %s", costmap_ros_->getGlobalFrameID().c_str());
}

bool Planner5D::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("Planner5D has not been initialized");
    return false;
  }

  global_plan_ = plan;
  goal_reached_ = false;

  if (global_plan_.empty())
  {
    ROS_WARN("Planner5D received an empty global plan");
    return false;
  }

  goal_pose_ = global_plan_.back();
  return true;
}

bool Planner5D::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  cmd_vel = geometry_msgs::Twist();

  if (!initialized_ || global_plan_.empty())
  {
    ROS_WARN_THROTTLE(1.0, "Planner5D cannot compute commands without initialization and a plan");
    return false;
  }

  const State5D current = getCurrentState();
  geometry_msgs::PoseStamped goal = goal_pose_;
  try
  {
    goal = tf_->transform(goal_pose_, costmap_ros_->getGlobalFrameID(), ros::Duration(0.1));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "Planner5D goal transform failed: %s", ex.what());
  }

  const double goal_x = goal.pose.position.x;
  const double goal_y = goal.pose.position.y;
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  const double goal_dist = distance2D(current.x, current.y, goal_x, goal_y);

  if (goal_dist <= xy_goal_tolerance_)
  {
    const double yaw_error = normalizeAngle(goal_yaw - current.theta);
    if (std::fabs(yaw_error) <= yaw_goal_tolerance_)
    {
      goal_reached_ = true;
      return true;
    }

    cmd_vel.angular.z = std::max(-max_vel_theta_, std::min(max_vel_theta_, 1.5 * yaw_error));
    return true;
  }

  geometry_msgs::PoseStamped target = goal;
  const double lookahead = std::min(0.8, std::max(0.35, goal_dist));
  double best_target_dist = std::numeric_limits<double>::infinity();
  for (const auto& pose : global_plan_)
  {
    geometry_msgs::PoseStamped transformed = pose;
    try
    {
      transformed = tf_->transform(pose, costmap_ros_->getGlobalFrameID(), ros::Duration(0.02));
    }
    catch (const tf2::TransformException&)
    {
      continue;
    }

    const double d = distance2D(current.x, current.y,
                               transformed.pose.position.x,
                               transformed.pose.position.y);
    const double lookahead_error = std::fabs(d - lookahead);
    if (lookahead_error < best_target_dist)
    {
      best_target_dist = lookahead_error;
      target = transformed;
    }
  }

  const double target_x = target.pose.position.x;
  const double target_y = target.pose.position.y;

  double best_score = std::numeric_limits<double>::infinity();
  double best_v = 0.0;
  double best_omega = 0.0;
  bool found = false;

  const int v_count = std::max(1, v_samples_);
  const int w_count = std::max(1, omega_samples_);
  for (int vi = 0; vi < v_count; ++vi)
  {
    const double alpha_v = (v_count == 1) ? 1.0 : static_cast<double>(vi) / (v_count - 1);
    const double v = min_vel_x_ + alpha_v * (max_vel_x_ - min_vel_x_);

    for (int wi = 0; wi < w_count; ++wi)
    {
      const double alpha_w = (w_count == 1) ? 0.5 : static_cast<double>(wi) / (w_count - 1);
      const double omega = -max_vel_theta_ + alpha_w * (2.0 * max_vel_theta_);

      State5D rollout = current;
      bool collision = false;
      for (double t = 0.0; t < sim_time_; t += sim_step_)
      {
        rollout = simulate(rollout, v, omega, sim_step_);
        if (isCollision(rollout))
        {
          collision = true;
          break;
        }
      }

      if (collision)
      {
        continue;
      }

      // Proximity penalty: discourages rolling out near obstacles even if not
      // strictly colliding.  costmap values range 0–252 in the inflation band.
      costmap_2d::Costmap2D* cm = costmap_ros_->getCostmap();
      unsigned int pmx = 0, pmy = 0;
      double proximity_penalty = 0.0;
      if (cm->worldToMap(rollout.x, rollout.y, pmx, pmy))
      {
        const unsigned char c = cm->getCost(pmx, pmy);
        if (c > 0 && c < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          // Keep the penalty gentle so the planner can still route near wall
          // ends when needed; the INSCRIBED collision check above handles safety.
          proximity_penalty = 0.3 * (static_cast<double>(c) / 252.0);
        }
      }

      const double target_dist    = distance2D(rollout.x, rollout.y, target_x, target_y);
      const double final_goal_dist = distance2D(rollout.x, rollout.y, goal_x, goal_y);
      const double desired_heading = std::atan2(target_y - rollout.y, target_x - rollout.x);
      const double heading_error   = std::fabs(normalizeAngle(desired_heading - rollout.theta));
      const double score = 4.0 * target_dist
                         + 1.2 * final_goal_dist
                         + 0.6 * heading_error
                         - 0.2 * v
                         + proximity_penalty;

      if (score < best_score)
      {
        best_score = score;
        best_v     = v;
        best_omega = omega;
        found = true;
      }
    }
  }

  if (!found)
  {
    // Alternate rotation direction every 1.2 s to escape symmetric dead-ends.
    const ros::Time now = ros::Time::now();
    if ((now - last_rotation_switch_).toSec() > 1.2)
    {
      rotation_sign_         = -rotation_sign_;
      last_rotation_switch_  = now;
    }
    ROS_WARN_THROTTLE(1.0, "Planner5D found no collision-free rollout; rotating in place (dir=%d)",
                      rotation_sign_);
    cmd_vel.angular.z = rotation_sign_ * 0.5 * max_vel_theta_;
    return true;
  }

  cmd_vel.linear.x = best_v;
  cmd_vel.angular.z = best_omega;
  return true;
}

bool Planner5D::isGoalReached()
{
  return goal_reached_;
}

State5D Planner5D::simulate(const State5D& s, double v, double omega, double dt) const
{
  State5D next = s;
  next.x += v * std::cos(s.theta) * dt;
  next.y += v * std::sin(s.theta) * dt;
  next.theta = normalizeAngle(s.theta + omega * dt);
  next.v = v;
  next.omega = omega;
  return next;
}

double Planner5D::heuristic(const State5D& s) const
{
  if (global_plan_.empty())
  {
    return 0.0;
  }
  return distance2D(s.x, s.y, goal_pose_.pose.position.x, goal_pose_.pose.position.y);
}

bool Planner5D::isCollision(const State5D& s) const
{
  if (!costmap_ros_)
  {
    return true;
  }

  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap->worldToMap(s.x, s.y, mx, my))
  {
    return true;
  }

  const unsigned char cost = costmap->getCost(mx, my);
  return cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

State5D Planner5D::getCurrentState() const
{
  geometry_msgs::PoseStamped pose;
  State5D state{};
  state.v = 0.0;
  state.omega = 0.0;

  if (!costmap_ros_->getRobotPose(pose))
  {
    ROS_WARN_THROTTLE(1.0, "Planner5D could not get robot pose");
    return state;
  }

  state.x = pose.pose.position.x;
  state.y = pose.pose.position.y;
  state.theta = tf2::getYaw(pose.pose.orientation);
  return state;
}

}  // namespace ctf_navigation

PLUGINLIB_EXPORT_CLASS(ctf_navigation::Planner5D, nav_core::BaseLocalPlanner)
