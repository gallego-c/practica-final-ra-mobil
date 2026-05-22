#include <algorithm>
#include <cmath>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
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
  double flag_capture_distance;
  double catch_distance;
  double chase_rate_hz;
  double goal_timeout;
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

void publishMarkers(ros::Publisher& pub,
                    const GameConfig& game,
                    const RobotConfig& robot1,
                    const RobotConfig& robot2)
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
  pub.publish(markers);
}

bool waitForServer(MoveBaseClient& client, const std::string& name)
{
  ROS_INFO("Waiting for %s move_base action server...", name.c_str());
  if (!client.waitForServer(ros::Duration(60.0)))
  {
    ROS_ERROR("Timed out waiting for %s move_base", name.c_str());
    return false;
  }
  return true;
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

int runSearchPhase(const GameConfig& game,
                   const RobotConfig robots[2],
                   MoveBaseClient* clients[2],
                   tf2_ros::Buffer& tf_buffer,
                   ros::Publisher& marker_pub)
{
  ROS_INFO("Phase 1: both robots search. %s and %s go to flag at (%.2f, %.2f)",
           robots[0].ns.c_str(), robots[1].ns.c_str(), game.flag_x, game.flag_y);

  for (int i = 0; i < 2; ++i)
  {
    clients[i]->sendGoal(makeGoal(game.frame, game.flag_x, game.flag_y, game.flag_yaw));
  }

  ros::Rate rate(std::max(0.2, game.chase_rate_hz));
  const ros::Time start = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();
    publishMarkers(marker_pub, game, robots[0], robots[1]);

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
    publishMarkers(marker_pub, game, robots[0], robots[1]);

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
  pnh.param("flag_capture_distance", game.flag_capture_distance, 0.45);
  pnh.param("catch_distance", game.catch_distance, 0.65);
  pnh.param("chase_rate_hz", game.chase_rate_hz, 1.0);
  pnh.param("goal_timeout", game.goal_timeout, 180.0);

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

  MoveBaseClient robot1_client("/" + robots[0].ns + "/move_base", true);
  MoveBaseClient robot2_client("/" + robots[1].ns + "/move_base", true);
  if (!waitForServer(robot1_client, robots[0].ns) ||
      !waitForServer(robot2_client, robots[1].ns))
  {
    return 1;
  }

  MoveBaseClient* clients[2] = {&robot1_client, &robot2_client};
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>("ctf_demo/markers", 1, true);
  ros::Duration(1.0).sleep();
  publishMarkers(marker_pub, game, robots[0], robots[1]);

  const int carrier_index = runSearchPhase(game, robots, clients, tf_buffer, marker_pub);
  if (carrier_index < 0)
  {
    return 2;
  }

  return runChasePhase(game, robots, clients, carrier_index, tf_buffer, marker_pub);
}
