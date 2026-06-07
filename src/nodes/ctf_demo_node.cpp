/**
 * @file ctf_demo_node.cpp
 * @brief Modo oráculo: ambos robots van directos a la bandera (sin visión).
 *        Útil para probar planificadores de movimiento.
 */
#include <algorithm>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "ctf_navigation/common/geometry.hpp"
#include "ctf_navigation/common/markers.hpp"
#include "ctf_navigation/common/move_base_client.hpp"
#include "ctf_navigation/common/slam_wait.hpp"
#include "ctf_navigation/common/tf_helper.hpp"
#include "ctf_navigation/game/types.hpp"

namespace
{

struct OracleConfig
{
  std::string frame = "map";
  double flag_x = -3.2;
  double flag_y = 3.2;
  double flag_yaw = 1.57;
  double flag_capture_distance = 0.45;
  double catch_distance = 0.65;
  double chase_rate_hz = 1.0;
  double goal_timeout = 180.0;

  ctf_navigation::game::HomePose robot1_home{-3.0, -3.0, 0.0};
  ctf_navigation::game::HomePose robot2_home{3.0, 3.0, 3.1416};
  std::string robot1_ns = "robot1";
  std::string robot2_ns = "robot2";
  std::string robot1_base_frame = "robot1/base_footprint";
  std::string robot2_base_frame = "robot2/base_footprint";
};

struct RobotMeta
{
  std::string ns;
  std::string base_frame;
  ctf_navigation::game::HomePose home;
};

void publishOracleMarkers(ros::Publisher& pub,
                          const OracleConfig& cfg,
                          const RobotMeta& r1,
                          const RobotMeta& r2)
{
  visualization_msgs::MarkerArray arr;
  arr.markers.push_back(ctf_navigation::markers::makeCylinder(
      cfg.frame, "ctf_flag", 0, cfg.flag_x, cfg.flag_y, 0.25, 0.8,
      1.0f, 0.1f, 0.1f, 0.9f));
  arr.markers.push_back(ctf_navigation::markers::makeCylinder(
      cfg.frame, "robot1_base", 1, r1.home.x, r1.home.y, 0.45, 0.03,
      0.1f, 0.5f, 1.0f, 0.8f));
  arr.markers.push_back(ctf_navigation::markers::makeCylinder(
      cfg.frame, "robot2_base", 2, r2.home.x, r2.home.y, 0.45, 0.03,
      1.0f, 0.6f, 0.1f, 0.8f));
  pub.publish(arr);
}

int runSearch(const OracleConfig& cfg,
              const RobotMeta& r0,
              const RobotMeta& r1,
              ctf_navigation::MoveBaseClientWrapper& mb0,
              ctf_navigation::MoveBaseClientWrapper& mb1,
              tf2_ros::Buffer& tf_buffer,
              ros::Publisher& marker_pub)
{
  ROS_INFO("Oracle search: go to flag (%.2f, %.2f)", cfg.flag_x, cfg.flag_y);
  mb0.sendGoal(cfg.frame, cfg.flag_x, cfg.flag_y, cfg.flag_yaw);
  mb1.sendGoal(cfg.frame, cfg.flag_x, cfg.flag_y, cfg.flag_yaw);

  ros::Rate rate(std::max(0.2, cfg.chase_rate_hz));
  const ros::Time start = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();
    publishOracleMarkers(marker_pub, cfg, r0, r1);

    geometry_msgs::TransformStamped t0, t1;
    const bool have0 = ctf_navigation::tf_helper::lookupTransform(
        tf_buffer, cfg.frame, r0.base_frame, t0);
    const bool have1 = ctf_navigation::tf_helper::lookupTransform(
        tf_buffer, cfg.frame, r1.base_frame, t1);

    if (have0 && have1)
    {
      const double d0 = ctf_navigation::geometry::distance2D(
          t0.transform.translation.x, t0.transform.translation.y, cfg.flag_x, cfg.flag_y);
      const double d1 = ctf_navigation::geometry::distance2D(
          t1.transform.translation.x, t1.transform.translation.y, cfg.flag_x, cfg.flag_y);
      ROS_INFO_THROTTLE(1.0, "Distance to flag: %s %.2f m, %s %.2f m",
                        r0.ns.c_str(), d0, r1.ns.c_str(), d1);
      if (d0 <= cfg.flag_capture_distance || d1 <= cfg.flag_capture_distance)
      {
        return (d0 <= d1) ? 0 : 1;
      }
    }

    if (mb0.isSucceeded() || mb1.isSucceeded())
    {
      return mb0.isSucceeded() ? 0 : 1;
    }

    if ((ros::Time::now() - start).toSec() > cfg.goal_timeout)
    {
      ROS_WARN("Search timeout");
      mb0.cancelAll();
      mb1.cancelAll();
      return -1;
    }
    rate.sleep();
  }
  return -1;
}

int runChase(const OracleConfig& cfg,
             const RobotMeta& r0,
             const RobotMeta& r1,
             ctf_navigation::MoveBaseClientWrapper& mb0,
             ctf_navigation::MoveBaseClientWrapper& mb1,
             int carrier_index,
             tf2_ros::Buffer& tf_buffer,
             ros::Publisher& marker_pub)
{
  if (carrier_index < 0 || carrier_index > 1)
  {
    ROS_ERROR("Invalid carrier index: %d", carrier_index);
    return -1;
  }
  const int pursuer_index = 1 - carrier_index;
  const RobotMeta& carrier_meta = (carrier_index == 0) ? r0 : r1;
  const RobotMeta& pursuer_meta = (pursuer_index == 0) ? r0 : r1;
  ctf_navigation::MoveBaseClientWrapper& carrier_mb = (carrier_index == 0) ? mb0 : mb1;
  ctf_navigation::MoveBaseClientWrapper& pursuer_mb = (pursuer_index == 0) ? mb0 : mb1;

  pursuer_mb.cancelAll();
  ros::Duration(0.5).sleep();

  ROS_INFO("Chase: %s → base, %s pursues", carrier_meta.ns.c_str(),
           pursuer_meta.ns.c_str());
  carrier_mb.sendGoal(cfg.frame, carrier_meta.home.x, carrier_meta.home.y,
                      carrier_meta.home.yaw);

  ros::Rate rate(std::max(0.2, cfg.chase_rate_hz));
  ros::Time last_chase_goal(0);

  while (ros::ok())
  {
    ros::spinOnce();
    publishOracleMarkers(marker_pub, cfg, r0, r1);

    if (carrier_mb.isSucceeded())
    {
      pursuer_mb.cancelAll();
      ROS_INFO("CTF: %s reached base", carrier_meta.ns.c_str());
      return 0;
    }

    geometry_msgs::TransformStamped tc, tp;
    if (!ctf_navigation::tf_helper::lookupTransform(
            tf_buffer, cfg.frame, carrier_meta.base_frame, tc) ||
        !ctf_navigation::tf_helper::lookupTransform(
            tf_buffer, cfg.frame, pursuer_meta.base_frame, tp))
    {
      rate.sleep();
      continue;
    }

    const double d = ctf_navigation::geometry::distanceBetween(tc, tp);
    ROS_INFO_THROTTLE(1.0, "Distance: %.2f m", d);
    if (d <= cfg.catch_distance)
    {
      carrier_mb.cancelAll();
      pursuer_mb.cancelAll();
      ROS_INFO("CTF: %s caught %s", pursuer_meta.ns.c_str(), carrier_meta.ns.c_str());
      return 0;
    }

    if ((ros::Time::now() - last_chase_goal).toSec() >=
        1.0 / std::max(0.2, cfg.chase_rate_hz))
    {
      pursuer_mb.sendGoal(cfg.frame, tc.transform.translation.x,
                          tc.transform.translation.y, 0.0);
      last_chase_goal = ros::Time::now();
    }
    rate.sleep();
  }
  return -1;
}

OracleConfig loadConfig(ros::NodeHandle& pnh)
{
  OracleConfig cfg;
  pnh.param("frame", cfg.frame, cfg.frame);
  pnh.param("flag_x", cfg.flag_x, cfg.flag_x);
  pnh.param("flag_y", cfg.flag_y, cfg.flag_y);
  pnh.param("flag_yaw", cfg.flag_yaw, cfg.flag_yaw);
  pnh.param("flag_capture_distance", cfg.flag_capture_distance,
            cfg.flag_capture_distance);
  pnh.param("catch_distance", cfg.catch_distance, cfg.catch_distance);
  pnh.param("chase_rate_hz", cfg.chase_rate_hz, cfg.chase_rate_hz);
  pnh.param("goal_timeout", cfg.goal_timeout, cfg.goal_timeout);
  pnh.param("robot1_ns", cfg.robot1_ns, cfg.robot1_ns);
  pnh.param("robot2_ns", cfg.robot2_ns, cfg.robot2_ns);
  pnh.param("robot1_base_frame", cfg.robot1_base_frame, cfg.robot1_base_frame);
  pnh.param("robot2_base_frame", cfg.robot2_base_frame, cfg.robot2_base_frame);
  pnh.param("robot1_base_x", cfg.robot1_home.x, cfg.robot1_home.x);
  pnh.param("robot1_base_y", cfg.robot1_home.y, cfg.robot1_home.y);
  pnh.param("robot1_base_yaw", cfg.robot1_home.yaw, cfg.robot1_home.yaw);
  pnh.param("robot2_base_x", cfg.robot2_home.x, cfg.robot2_home.x);
  pnh.param("robot2_base_y", cfg.robot2_home.y, cfg.robot2_home.y);
  pnh.param("robot2_base_yaw", cfg.robot2_home.yaw, cfg.robot2_home.yaw);
  return cfg;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ctf_demo_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  const OracleConfig cfg = loadConfig(pnh);

  double navigation_startup_delay = 0.0;
  double move_base_wait_timeout = 120.0;
  std::string wait_for_map_topic;
  double map_wait_timeout = 90.0;
  int map_min_width = 80;
  int map_min_known_cells = 800;
  std::string wait_for_exploration_topic;
  double exploration_wait_timeout = 320.0;
  double startup_delay = 0.0;

  pnh.param("navigation_startup_delay", navigation_startup_delay, navigation_startup_delay);
  pnh.param("move_base_wait_timeout", move_base_wait_timeout, move_base_wait_timeout);
  pnh.param("wait_for_map_topic", wait_for_map_topic, wait_for_map_topic);
  pnh.param("map_wait_timeout", map_wait_timeout, map_wait_timeout);
  pnh.param("map_min_width", map_min_width, map_min_width);
  pnh.param("map_min_known_cells", map_min_known_cells, map_min_known_cells);
  pnh.param("wait_for_exploration_topic", wait_for_exploration_topic,
            wait_for_exploration_topic);
  pnh.param("exploration_wait_timeout", exploration_wait_timeout, exploration_wait_timeout);
  pnh.param("startup_delay", startup_delay, startup_delay);

  if (navigation_startup_delay > 0.0)
  {
    ROS_INFO("Navigation startup delay: %.1f s", navigation_startup_delay);
    ros::Duration(navigation_startup_delay).sleep();
  }

  const RobotMeta meta0{cfg.robot1_ns, cfg.robot1_base_frame, cfg.robot1_home};
  const RobotMeta meta1{cfg.robot2_ns, cfg.robot2_base_frame, cfg.robot2_home};

  ctf_navigation::MoveBaseClientWrapper mb0(cfg.robot1_ns);
  ctf_navigation::MoveBaseClientWrapper mb1(cfg.robot2_ns);

  if (!mb0.waitForServer(move_base_wait_timeout) ||
      !mb1.waitForServer(move_base_wait_timeout))
  {
    return 1;
  }

  if (startup_delay > 0.0)
  {
    ros::Duration(startup_delay).sleep();
  }

  if (!ctf_navigation::slam_wait::waitForNavigationMap(
          wait_for_map_topic, map_wait_timeout,
          static_cast<unsigned int>(map_min_width),
          static_cast<unsigned int>(map_min_known_cells)))
  {
    return 2;
  }

  ctf_navigation::slam_wait::waitForExplorationComplete(wait_for_exploration_topic,
                                                        exploration_wait_timeout);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  auto marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "ctf_demo/markers", 1, true);
  ros::Duration(1.0).sleep();
  publishOracleMarkers(marker_pub, cfg, meta0, meta1);

  const int carrier = runSearch(cfg, meta0, meta1, mb0, mb1, tf_buffer, marker_pub);
  if (carrier < 0)
  {
    return 2;
  }
  return runChase(cfg, meta0, meta1, mb0, mb1, carrier, tf_buffer, marker_pub);
}
