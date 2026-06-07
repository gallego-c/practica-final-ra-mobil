#ifndef CTF_NAVIGATION_COMMON_MOVE_BASE_CLIENT_HPP
#define CTF_NAVIGATION_COMMON_MOVE_BASE_CLIENT_HPP

#include <string>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

#include "ctf_navigation/common/geometry.hpp"

namespace ctf_navigation
{

class MoveBaseClientWrapper
{
public:
  MoveBaseClientWrapper(const MoveBaseClientWrapper&) = delete;
  MoveBaseClientWrapper& operator=(const MoveBaseClientWrapper&) = delete;
  MoveBaseClientWrapper(MoveBaseClientWrapper&&) = delete;
  MoveBaseClientWrapper& operator=(MoveBaseClientWrapper&&) = delete;

  explicit MoveBaseClientWrapper(const std::string& robot_ns)
    : client_("/" + robot_ns + "/move_base", true)
    , robot_ns_(robot_ns)
  {
  }

  bool waitForServer(double timeout_sec = 60.0)
  {
    ROS_INFO("[%s] Waiting for move_base...", robot_ns_.c_str());
    if (!client_.waitForServer(ros::Duration(timeout_sec)))
    {
      ROS_ERROR("[%s] move_base not available", robot_ns_.c_str());
      return false;
    }
    return true;
  }

  void sendGoal(const std::string& frame, double x, double y, double yaw)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = frame;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = geometry::yawToQuaternion(yaw);
    client_.sendGoal(goal);
  }

  void cancelAll()
  {
    client_.cancelAllGoals();
  }

  actionlib::SimpleClientGoalState getState() const
  {
    return client_.getState();
  }

  bool isSucceeded() const
  {
    return getState() ==
           actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::SUCCEEDED);
  }

  bool isTerminal() const
  {
    const auto s = getState();
    return s == actionlib::SimpleClientGoalState::SUCCEEDED ||
           s == actionlib::SimpleClientGoalState::ABORTED ||
           s == actionlib::SimpleClientGoalState::REJECTED;
  }

  std::string stateText() const
  {
    return getState().toString();
  }

private:
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client_;
  std::string robot_ns_;
};

}  // namespace ctf_navigation

#endif
