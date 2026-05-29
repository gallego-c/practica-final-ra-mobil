#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

namespace
{

struct RobotConfig
{
  std::string ns;
  std::string base_frame;
  double home_x;
  double home_y;
  double home_yaw;
};

struct GameConfig
{
  std::string frame;
  double flag_x;
  double flag_y;
  double flag_yaw;
  double search_goal_separation;
  double flag_capture_distance;
  double catch_distance;
  double chase_rate_hz;
  double goal_timeout;
  bool guided_exploration;
  double exploration_goal_timeout;
  std::string wait_for_exploration_topic;
  double exploration_wait_timeout;
};

struct Waypoint
{
  double x;
  double y;
  double yaw;
};

geometry_msgs::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

move_base_msgs::MoveBaseGoal makeGoal(const std::string& frame,
                                      double x,
                                      double y,
                                      double yaw)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = frame;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation = yawToQuaternion(yaw);
  return goal;
}

bool isSucceeded(const actionlib::SimpleClientGoalState& state)
{
  return state == actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::SUCCEEDED);
}

double distance2D(double x0, double y0, double x1, double y1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  return std::sqrt(dx * dx + dy * dy);
}

double distanceBetween(const geometry_msgs::TransformStamped& a,
                       const geometry_msgs::TransformStamped& b)
{
  return distance2D(a.transform.translation.x, a.transform.translation.y,
                    b.transform.translation.x, b.transform.translation.y);
}

double distanceToPoint(const geometry_msgs::TransformStamped& transform,
                       double x,
                       double y)
{
  return distance2D(transform.transform.translation.x, transform.transform.translation.y, x, y);
}

move_base_msgs::MoveBaseGoal makeApproachGoal(const GameConfig& game,
                                              const RobotConfig& robot,
                                              int robot_index)
{
  if (game.search_goal_separation <= 0.0)
  {
    return makeGoal(game.frame, game.flag_x, game.flag_y, game.flag_yaw);
  }

  double dx = robot.home_x - game.flag_x;
  double dy = robot.home_y - game.flag_y;
  const double length = std::sqrt(dx * dx + dy * dy);
  if (length < 1e-3)
  {
    dx = (robot_index == 0) ? -1.0 : 1.0;
    dy = 0.0;
  }
  else
  {
    dx /= length;
    dy /= length;
  }

  const double side = (robot_index == 0) ? 1.0 : -1.0;
  const double offset_x = dx * game.search_goal_separation;
  const double offset_y = dy * game.search_goal_separation;
  const double lateral_x = -dy * game.search_goal_separation * 0.5 * side;
  const double lateral_y = dx * game.search_goal_separation * 0.5 * side;

  return makeGoal(game.frame,
                  game.flag_x + offset_x + lateral_x,
                  game.flag_y + offset_y + lateral_y,
                  game.flag_yaw);
}

visualization_msgs::Marker makeCylinderMarker(const std::string& frame,
                                              const std::string& ns,
                                              int id,
                                              double x,
                                              double y,
                                              double radius,
                                              double height,
                                              float r,
                                              float g,
                                              float b,
                                              float a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = height * 0.5;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = height;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.lifetime = ros::Duration(0.0);
  return marker;
}

visualization_msgs::Marker makeRobotPoseMarker(const std::string& frame,
                                               const std::string& ns,
                                               int id,
                                               const geometry_msgs::TransformStamped& pose,
                                               float r,
                                               float g,
                                               float b)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pose.transform.translation.x;
  marker.pose.position.y = pose.transform.translation.y;
  marker.pose.position.z = 0.15;
  marker.pose.orientation = pose.transform.rotation;
  marker.scale.x = 0.45;
  marker.scale.y = 0.08;
  marker.scale.z = 0.08;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration(0.5);
  return marker;
}

void publishMarkers(ros::Publisher& pub,
                    const GameConfig& game,
                    const RobotConfig& robot1,
                    const RobotConfig& robot2,
                    tf2_ros::Buffer* tf_buffer = nullptr)
{
  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(makeCylinderMarker(game.frame, "ctf_flag", 0,
                                               game.flag_x, game.flag_y,
                                               0.25, 0.8, 1.0f, 0.1f, 0.1f, 0.9f));
  markers.markers.push_back(makeCylinderMarker(game.frame, "robot1_base", 1,
                                               robot1.home_x, robot1.home_y,
                                               0.45, 0.03, 0.1f, 0.5f, 1.0f, 0.8f));
  markers.markers.push_back(makeCylinderMarker(game.frame, "robot2_base", 2,
                                               robot2.home_x, robot2.home_y,
                                               0.45, 0.03, 1.0f, 0.6f, 0.1f, 0.8f));

  if (tf_buffer)
  {
    try
    {
      const auto robot1_pose = tf_buffer->lookupTransform(
          game.frame, robot1.base_frame, ros::Time(0), ros::Duration(0.02));
      markers.markers.push_back(makeRobotPoseMarker(game.frame, "robot1_pose", 3,
                                                    robot1_pose, 0.1f, 0.5f, 1.0f));
    }
    catch (const tf2::TransformException&)
    {
    }

    try
    {
      const auto robot2_pose = tf_buffer->lookupTransform(
          game.frame, robot2.base_frame, ros::Time(0), ros::Duration(0.02));
      markers.markers.push_back(makeRobotPoseMarker(game.frame, "robot2_pose", 4,
                                                    robot2_pose, 1.0f, 0.6f, 0.1f));
    }
    catch (const tf2::TransformException&)
    {
    }
  }

  pub.publish(markers);
}

bool waitForServer(MoveBaseClient& client, const std::string& name, double timeout_sec)
{
  ROS_INFO("Waiting for %s move_base action server (%.0f s)...",
           name.c_str(), timeout_sec);
  if (!client.waitForServer(ros::Duration(timeout_sec)))
  {
    ROS_ERROR("Timed out waiting for %s move_base", name.c_str());
    return false;
  }
  return true;
}

bool waitForNavigationMap(const std::string& topic,
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

bool waitForExplorationComplete(const std::string& topic, double timeout_sec)
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

bool lookupRobotPose(tf2_ros::Buffer& tf_buffer,
                     const GameConfig& game,
                     const RobotConfig& robot,
                     geometry_msgs::TransformStamped& pose)
{
  try
  {
    pose = tf_buffer.lookupTransform(game.frame, robot.base_frame, ros::Time(0),
                                     ros::Duration(0.2));
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "Waiting for %s transform: %s", robot.ns.c_str(), ex.what());
    return false;
  }
}

bool waitForRobotPoses(tf2_ros::Buffer& tf_buffer,
                       const GameConfig& game,
                       const RobotConfig robots[2],
                       double timeout_sec)
{
  const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
  ros::Rate rate(5.0);

  while (ros::ok() && ros::Time::now() < deadline)
  {
    geometry_msgs::TransformStamped poses[2];
    if (lookupRobotPose(tf_buffer, game, robots[0], poses[0]) &&
        lookupRobotPose(tf_buffer, game, robots[1], poses[1]))
    {
      const double separation = distanceBetween(poses[0], poses[1]);
      ROS_INFO("Both robots localized in %s (separation %.2f m)",
               game.frame.c_str(), separation);
      return true;
    }
    rate.sleep();
  }

  ROS_WARN("Timed out waiting for both robot poses in %s", game.frame.c_str());
  return false;
}

bool waitForGoal(MoveBaseClient& client,
                 double timeout_sec,
                 const std::string& label,
                 ros::Publisher& marker_pub,
                 const GameConfig& game,
                 const RobotConfig robots[2],
                 tf2_ros::Buffer& tf_buffer)
{
  const ros::Time start = ros::Time::now();
  ros::Rate rate(std::max(2.0, game.chase_rate_hz));
  while (ros::ok())
  {
    ros::spinOnce();
    publishMarkers(marker_pub, game, robots[0], robots[1], &tf_buffer);

    const auto state = client.getState();
    if (state.isDone())
    {
      if (isSucceeded(state))
      {
        ROS_INFO("%s reached", label.c_str());
        return true;
      }

      ROS_WARN("%s finished with state %s", label.c_str(), state.toString().c_str());
      return false;
    }

    if ((ros::Time::now() - start).toSec() > timeout_sec)
    {
      ROS_WARN("%s timed out; cancelling and continuing", label.c_str());
      client.cancelGoal();
      return false;
    }

    rate.sleep();
  }

  return false;
}

void runGuidedExplorationPhase(const GameConfig& game,
                               const RobotConfig robots[2],
                               MoveBaseClient* clients[2],
                               tf2_ros::Buffer& tf_buffer,
                               ros::Publisher& marker_pub)
{
  if (!game.guided_exploration)
  {
    return;
  }

  ROS_INFO("Phase 0: parallel guided SLAM exploration");

  // robot1 starts bottom-left (-3,-3).  Route: explore own zone → approach gap
  //   left side → cross to north zone → reach flag area.
  // robot2 starts top-right (3,3).  Route: explore own zone → approach gap
  //   right side → help map south zone → converge on flag area.
  // Robots pass on opposite sides of the 1-m gap (x ∈ [-1,1] @ y=0) to
  // minimise the chance of a head-on collision.
  //
  // Obstacle reference (inflation 0.28 m):
  //   col_1 centre (-1.0, 1.5) 0.3×0.3 → keep x > -0.57 near y=1.5
  //   col_2 centre ( 1.0,-1.5) 0.3×0.3 → keep x <  0.57 near y=-1.5
  const std::vector<Waypoint> r1 = {
      {-2.5, -1.5,  0.00},   // away from spawn, clear of obs_1 / obs_2
      {-0.6, -0.3,  1.57},   // south mouth of gap, left lane
      {-0.3,  1.5,  1.57},   // north side, east of col_1 — maps flag corridor
      {-2.8,  3.0,  1.57},   // near flag area
  };
  const std::vector<Waypoint> r2 = {
      { 2.5,  1.5,  3.14},   // away from spawn, clear of obs_3 / obs_4
      { 0.6,  0.3, -1.57},   // north mouth of gap, right lane
      { 0.3, -1.5, -1.57},   // south side, west of col_2 — helps map robot1 zone
      {-2.5,  2.8,  2.36},   // also approach flag area
  };

  const std::size_t n = std::max(r1.size(), r2.size());
  ros::Rate rate(5.0);

  for (std::size_t i = 0; i < n; ++i)
  {
    // ── Send goals to BOTH robots simultaneously ──────────────────────────
    if (i < r1.size())
    {
      const auto& wp = r1[i];
      ROS_INFO("Exploration R1[%zu]: %.2f, %.2f", i, wp.x, wp.y);
      clients[0]->sendGoal(makeGoal(game.frame, wp.x, wp.y, wp.yaw));
    }
    if (i < r2.size())
    {
      const auto& wp = r2[i];
      ROS_INFO("Exploration R2[%zu]: %.2f, %.2f", i, wp.x, wp.y);
      clients[1]->sendGoal(makeGoal(game.frame, wp.x, wp.y, wp.yaw));
    }

    // ── Wait for BOTH to finish (or time-out) in a shared loop ────────────
    const ros::Time deadline =
        ros::Time::now() + ros::Duration(game.exploration_goal_timeout);

    while (ros::ok() && ros::Time::now() < deadline)
    {
      ros::spinOnce();
      publishMarkers(marker_pub, game, robots[0], robots[1], &tf_buffer);

      const bool r1_pending = (i < r1.size()) && !clients[0]->getState().isDone();
      const bool r2_pending = (i < r2.size()) && !clients[1]->getState().isDone();
      if (!r1_pending && !r2_pending)
        break;

      rate.sleep();
    }

    if (i < r1.size() && !clients[0]->getState().isDone())
    {
      ROS_WARN("Exploration R1[%zu] timed out, cancelling", i);
      clients[0]->cancelGoal();
    }
    if (i < r2.size() && !clients[1]->getState().isDone())
    {
      ROS_WARN("Exploration R2[%zu] timed out, cancelling", i);
      clients[1]->cancelGoal();
    }

    ros::Duration(0.5).sleep();
  }

  ROS_INFO("Phase 0: guided exploration complete");
}

int runSearchPhase(const GameConfig& game,
                   const RobotConfig robots[2],
                   MoveBaseClient* clients[2],
                   tf2_ros::Buffer& tf_buffer,
                   ros::Publisher& marker_pub)
{
  ROS_INFO("Phase 1: both robots search. %s and %s go to flag at (%.2f, %.2f)",
           robots[0].ns.c_str(), robots[1].ns.c_str(), game.flag_x, game.flag_y);

  clients[0]->sendGoal(makeApproachGoal(game, robots[0], 0));
  ros::Duration(2.0).sleep();
  clients[1]->sendGoal(makeApproachGoal(game, robots[1], 1));

  ros::Rate rate(std::max(0.2, game.chase_rate_hz));
  const ros::Time start = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();
    publishMarkers(marker_pub, game, robots[0], robots[1], &tf_buffer);

    bool captured_by_action[2] = {isSucceeded(clients[0]->getState()),
                                  isSucceeded(clients[1]->getState())};

    geometry_msgs::TransformStamped poses[2];
    bool have_pose[2] = {lookupRobotPose(tf_buffer, game, robots[0], poses[0]),
                         lookupRobotPose(tf_buffer, game, robots[1], poses[1])};

    if (have_pose[0] && have_pose[1])
    {
      const double flag_distances[2] = {
          distanceToPoint(poses[0], game.flag_x, game.flag_y),
          distanceToPoint(poses[1], game.flag_x, game.flag_y)};

      ROS_INFO_THROTTLE(1.0, "Search distance to flag: %s %.2f m, %s %.2f m",
                        robots[0].ns.c_str(), flag_distances[0],
                        robots[1].ns.c_str(), flag_distances[1]);

      if (flag_distances[0] <= game.flag_capture_distance ||
          flag_distances[1] <= game.flag_capture_distance)
      {
        return (flag_distances[0] <= flag_distances[1]) ? 0 : 1;
      }
    }

    if (captured_by_action[0] || captured_by_action[1])
    {
      return captured_by_action[0] ? 0 : 1;
    }

    if ((ros::Time::now() - start).toSec() > game.goal_timeout)
    {
      ROS_WARN("No robot reached the flag before timeout; cancelling search goals");
      clients[0]->cancelGoal();
      clients[1]->cancelGoal();
      return -1;
    }

    rate.sleep();
  }

  return -1;
}

int runChasePhase(const GameConfig& game,
                  const RobotConfig robots[2],
                  MoveBaseClient* clients[2],
                  int carrier_index,
                  tf2_ros::Buffer& tf_buffer,
                  ros::Publisher& marker_pub)
{
  const int pursuer_index = 1 - carrier_index;
  const RobotConfig& carrier = robots[carrier_index];
  const RobotConfig& pursuer = robots[pursuer_index];
  MoveBaseClient& carrier_client = *clients[carrier_index];
  MoveBaseClient& pursuer_client = *clients[pursuer_index];

  pursuer_client.cancelGoal();
  ros::Duration(0.5).sleep();

  ROS_INFO("Phase 2: flag captured by %s. %s returns to base and %s chases.",
           carrier.ns.c_str(), carrier.ns.c_str(), pursuer.ns.c_str());
  carrier_client.sendGoal(makeGoal(game.frame, carrier.home_x, carrier.home_y, carrier.home_yaw));

  ros::Rate rate(std::max(0.2, game.chase_rate_hz));
  ros::Time last_chase_goal(0);

  while (ros::ok())
  {
    ros::spinOnce();
    publishMarkers(marker_pub, game, robots[0], robots[1], &tf_buffer);

    if (carrier_client.getState().isDone())
    {
      if (isSucceeded(carrier_client.getState()))
      {
        pursuer_client.cancelGoal();
        ROS_INFO("CTF result: %s reached base before being caught", carrier.ns.c_str());
        return 0;
      }
      return 3;
    }

    geometry_msgs::TransformStamped carrier_pose;
    geometry_msgs::TransformStamped pursuer_pose;
    if (!lookupRobotPose(tf_buffer, game, carrier, carrier_pose) ||
        !lookupRobotPose(tf_buffer, game, pursuer, pursuer_pose))
    {
      rate.sleep();
      continue;
    }

    const double robot_distance = distanceBetween(carrier_pose, pursuer_pose);
    ROS_INFO_THROTTLE(1.0, "Distance %s-%s: %.2f m",
                      carrier.ns.c_str(), pursuer.ns.c_str(), robot_distance);

    if (robot_distance <= game.catch_distance)
    {
      carrier_client.cancelGoal();
      pursuer_client.cancelGoal();
      ROS_INFO("CTF result: %s caught %s within %.2f m",
               pursuer.ns.c_str(), carrier.ns.c_str(), game.catch_distance);
      return 0;
    }

    if ((ros::Time::now() - last_chase_goal).toSec() >= 1.0 / std::max(0.2, game.chase_rate_hz))
    {
      pursuer_client.sendGoal(makeGoal(game.frame,
                                       carrier_pose.transform.translation.x,
                                       carrier_pose.transform.translation.y,
                                       0.0));
      last_chase_goal = ros::Time::now();
    }

    rate.sleep();
  }

  return 3;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ctf_demo_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  GameConfig game;
  pnh.param<std::string>("frame", game.frame, "map");
  pnh.param("flag_x", game.flag_x, -3.2);
  pnh.param("flag_y", game.flag_y, 3.2);
  pnh.param("flag_yaw", game.flag_yaw, 1.57);
  pnh.param("search_goal_separation", game.search_goal_separation, 0.0);
  pnh.param("flag_capture_distance", game.flag_capture_distance, 0.45);
  pnh.param("catch_distance", game.catch_distance, 0.65);
  pnh.param("chase_rate_hz", game.chase_rate_hz, 1.0);
  pnh.param("goal_timeout", game.goal_timeout, 180.0);
  pnh.param("guided_exploration", game.guided_exploration, false);
  pnh.param("exploration_goal_timeout", game.exploration_goal_timeout, 45.0);
  pnh.param<std::string>("wait_for_exploration_topic", game.wait_for_exploration_topic, "");
  pnh.param("exploration_wait_timeout", game.exploration_wait_timeout, 150.0);

  std::string wait_for_map_topic;
  pnh.param<std::string>("wait_for_map_topic", wait_for_map_topic, "");
  double map_wait_timeout = wait_for_map_topic.empty() ? 0.0 : 90.0;
  pnh.param("map_wait_timeout", map_wait_timeout, map_wait_timeout);
  int map_min_width = 80;
  pnh.param("map_min_width", map_min_width, 80);
  int map_min_known_cells = 500;
  pnh.param("map_min_known_cells", map_min_known_cells, 500);
  double startup_delay = 0.0;
  pnh.param("startup_delay", startup_delay, 0.0);
  double pose_wait_timeout = 0.0;
  pnh.param("pose_wait_timeout", pose_wait_timeout, 0.0);
  double move_base_wait_timeout = 120.0;
  pnh.param("move_base_wait_timeout", move_base_wait_timeout, 120.0);
  double navigation_startup_delay = 0.0;
  pnh.param("navigation_startup_delay", navigation_startup_delay, 0.0);

  RobotConfig robots[2];
  pnh.param<std::string>("robot1_ns", robots[0].ns, "robot1");
  pnh.param<std::string>("robot1_base_frame", robots[0].base_frame, "robot1/base_footprint");
  pnh.param("robot1_base_x", robots[0].home_x, -3.0);
  pnh.param("robot1_base_y", robots[0].home_y, -3.0);
  pnh.param("robot1_base_yaw", robots[0].home_yaw, 0.0);

  pnh.param<std::string>("robot2_ns", robots[1].ns, "robot2");
  pnh.param<std::string>("robot2_base_frame", robots[1].base_frame, "robot2/base_footprint");
  pnh.param("robot2_base_x", robots[1].home_x, 3.0);
  pnh.param("robot2_base_y", robots[1].home_y, 3.0);
  pnh.param("robot2_base_yaw", robots[1].home_yaw, 3.1416);

  if (navigation_startup_delay > 0.0)
  {
    ROS_INFO("Navigation startup delay: %.1f s", navigation_startup_delay);
    ros::Duration(navigation_startup_delay).sleep();
  }

  MoveBaseClient robot1_client("/" + robots[0].ns + "/move_base", true);
  MoveBaseClient robot2_client("/" + robots[1].ns + "/move_base", true);
  if (!waitForServer(robot1_client, robots[0].ns, move_base_wait_timeout) ||
      !waitForServer(robot2_client, robots[1].ns, move_base_wait_timeout))
  {
    return 1;
  }

  MoveBaseClient* clients[2] = {&robot1_client, &robot2_client};
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>("ctf_demo/markers", 1, true);

  if (startup_delay > 0.0)
  {
    ROS_INFO("Startup delay: %.1f s before sending goals", startup_delay);
    ros::Duration(startup_delay).sleep();
  }

  if (!wait_for_map_topic.empty() &&
      !waitForNavigationMap(wait_for_map_topic, map_wait_timeout,
                            static_cast<unsigned int>(map_min_width),
                            static_cast<unsigned int>(map_min_known_cells)))
  {
    return 2;
  }

  if (pose_wait_timeout > 0.0 &&
      !waitForRobotPoses(tf_buffer, game, robots, pose_wait_timeout))
  {
    return 2;
  }

  if (!game.wait_for_exploration_topic.empty())
  {
    waitForExplorationComplete(game.wait_for_exploration_topic,
                               game.exploration_wait_timeout);
  }

  ros::Duration(1.0).sleep();
  publishMarkers(marker_pub, game, robots[0], robots[1], &tf_buffer);

  runGuidedExplorationPhase(game, robots, clients, tf_buffer, marker_pub);

  const int carrier_index = runSearchPhase(game, robots, clients, tf_buffer, marker_pub);
  if (carrier_index < 0)
  {
    return 2;
  }

  return runChasePhase(game, robots, clients, carrier_index, tf_buffer, marker_pub);
}
