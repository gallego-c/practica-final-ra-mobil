#ifndef CTF_NAVIGATION_COMMON_SLAM_WAIT_HPP
#define CTF_NAVIGATION_COMMON_SLAM_WAIT_HPP

#include <string>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace ctf_navigation
{
namespace slam_wait
{

inline bool waitForNavigationMap(const std::string& topic,
                                 double timeout_sec,
                                 unsigned int min_width,
                                 unsigned int min_known_cells)
{
  if (topic.empty())
  {
    return true;
  }

  nav_msgs::OccupancyGrid::ConstPtr map;
  ros::NodeHandle nh;
  const ros::Subscriber sub =
      nh.subscribe<nav_msgs::OccupancyGrid>(topic, 1,
                                            [&map](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
                                              map = msg;
                                            });

  ROS_INFO("Waiting for navigation map on %s...", topic.c_str());
  const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
  ros::Rate rate(5.0);

  while (ros::ok() && ros::Time::now() < deadline)
  {
    ros::spinOnce();
    if (map && map->info.width >= min_width && map->info.height >= min_width)
    {
      int known_cells = 0;
      for (const int value : map->data)
      {
        if (value >= 0)
        {
          ++known_cells;
        }
      }

      if (static_cast<unsigned int>(known_cells) >= min_known_cells)
      {
        ROS_INFO("Navigation map ready on %s (%ux%u, %d known cells)",
                 topic.c_str(), map->info.width, map->info.height, known_cells);
        return true;
      }
    }
    rate.sleep();
  }

  ROS_WARN("Timed out waiting for navigation map on %s", topic.c_str());
  return false;
}

inline bool waitForExplorationComplete(const std::string& topic, double timeout_sec)
{
  if (topic.empty())
  {
    return true;
  }

  bool done = false;
  ros::NodeHandle nh;
  const ros::Subscriber sub =
      nh.subscribe<std_msgs::Bool>(topic, 1,
                                   [&done](const std_msgs::Bool::ConstPtr& msg) {
                                     if (msg->data)
                                     {
                                       done = true;
                                     }
                                   });

  ROS_INFO("Waiting for frontier exploration on %s...", topic.c_str());
  const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
  ros::Rate rate(5.0);

  while (ros::ok() && ros::Time::now() < deadline)
  {
    ros::spinOnce();
    if (done)
    {
      ROS_INFO("Frontier exploration complete");
      return true;
    }
    rate.sleep();
  }

  ROS_WARN("Timed out waiting for exploration on %s; continuing anyway", topic.c_str());
  return false;
}

}  
}  

#endif
