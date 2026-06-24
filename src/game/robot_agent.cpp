#include "ctf_navigation/game/robot_agent.hpp"

#include "ctf_navigation/common/geometry.hpp"

namespace ctf_navigation
{
namespace game
{

RobotAgent::RobotAgent(const std::string& ns,
                       const std::string& base_frame,
                       const HomePose& home,
                       const std::string& map_frame,
                       const std::string& detector_node,
                       tf2_ros::Buffer& tf_buffer)
  : ns_(ns)
  , base_frame_(base_frame)
  , map_frame_(map_frame)
  , home_(home)
  , move_base_(ns)
  , tf_buffer_(tf_buffer)
  , target_waypoint_(-1)
{
  const std::string base = "/" + ns + "/" + detector_node;
  ros::NodeHandle nh;
  flag_found_sub_ = nh.subscribe(base + "/flag_found", 1, &RobotAgent::onFlagFound, this);
  flag_estimate_sub_ =
      nh.subscribe(base + "/flag_estimate", 1, &RobotAgent::onFlagEstimate, this);
}

bool RobotAgent::waitForMoveBase(double timeout_sec)
{
  return move_base_.waitForServer(timeout_sec);
}

boost::optional<tf_helper::Pose2D> RobotAgent::poseInMap() const
{
  return tf_helper::lookupPose2D(tf_buffer_, map_frame_, base_frame_);
}

void RobotAgent::sendGoal(double x, double y, double yaw)
{
  move_base_.sendGoal(map_frame_, x, y, yaw);
}

void RobotAgent::cancelGoals()
{
  move_base_.cancelAll();
}

bool RobotAgent::moveBaseSucceeded() const
{
  return move_base_.isSucceeded();
}

bool RobotAgent::moveBaseTerminal() const
{
  return move_base_.isTerminal();
}

bool RobotAgent::hasFreshFlagEstimate(double timeout_sec) const
{
  if (!flag_estimate_)
  {
    return false;
  }
  return (ros::Time::now() - flag_estimate_time_).toSec() <= timeout_sec;
}

boost::optional<std::pair<double, double>> RobotAgent::flagEstimate() const
{
  return flag_estimate_;
}

void RobotAgent::resetFlagGoalThrottle()
{
  last_flag_goal_ = boost::none;
}

void RobotAgent::resetForPhaseTransition()
{
  search_state_ = SearchState::EXPLORING;
  target_waypoint_ = -1;
  resetFlagGoalThrottle();
  cancelGoals();
}

bool RobotAgent::shouldSendFlagGoal(double fx, double fy,
                                    double min_move_m, double max_age_sec)
{
  const ros::Time now = ros::Time::now();
  const bool moved = !last_flag_goal_ ||
                     geometry::distance2D(fx, fy, last_flag_goal_->first,
                                          last_flag_goal_->second) > min_move_m;
  const bool stale = (now - last_flag_goal_time_).toSec() > max_age_sec;
  if (moved || stale)
  {
    last_flag_goal_ = std::make_pair(fx, fy);
    last_flag_goal_time_ = now;
    return true;
  }
  return false;
}

void RobotAgent::onFlagFound(const std_msgs::Bool::ConstPtr& msg)
{
  flag_seen_ = msg->data;
}

void RobotAgent::onFlagEstimate(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  flag_estimate_ = std::make_pair(msg->pose.position.x, msg->pose.position.y);
  flag_estimate_time_ = ros::Time::now();
}

}  // namespace game
}  // namespace ctf_navigation
