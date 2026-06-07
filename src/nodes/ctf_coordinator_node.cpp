/**
 * @file ctf_coordinator_node.cpp
 * @brief Coordinador del juego CTF: exploración, visión, captura y persecución.
 */
#include <algorithm>
#include <cstddef>
#include <cmath>
#include <string>

#include <boost/optional.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "ctf_navigation/common/geometry.hpp"
#include "ctf_navigation/common/markers.hpp"
#include "ctf_navigation/game/exploration.hpp"
#include "ctf_navigation/game/robot_agent.hpp"

namespace
{

struct CoordinatorConfig
{
  std::string map_frame = "map";
  std::string map_topic = "/map";
  std::string detector_node = "flag_detector";
  double rate_hz = 5.0;
  double flag_capture_distance = 0.40;
  double flag_standoff_distance = 0.20;
  double capture_pause_sec = 3.0;
  double catch_distance = 0.65;
  double flag_memory_timeout = 5.0;
  double waypoint_reached_dist = 0.55;
  double chase_goal_period = 1.0;
  double explore_step = 1.0;
  double explore_clearance = 0.30;
  double map_wait_timeout = 60.0;
  double trail_min_distance = 0.05;
  int trail_max_poses = 2000;
  bool intercept_enabled = true;
  double intercept_direct_chase_distance = 0.85;
  double intercept_arrival_margin = 0.30;
  double intercept_step = 0.25;
  double intercept_min_lead = 1.00;

  ctf_navigation::game::HomePose robot1_home{-3.0, -3.0, 0.0};
  ctf_navigation::game::HomePose robot2_home{3.0, 3.0, 3.1416};
  std::string robot1_ns = "robot1";
  std::string robot2_ns = "robot2";
  std::string robot1_base_frame = "robot1/base_footprint";
  std::string robot2_base_frame = "robot2/base_footprint";
};

class CtfCoordinator
{
public:
  explicit CtfCoordinator(const CoordinatorConfig& cfg)
    : cfg_(cfg)
    , tf_listener_(tf_buffer_)
    , exploration_()
    , agent0_(cfg.robot1_ns, cfg.robot1_base_frame, cfg.robot1_home, cfg.map_frame,
              cfg.detector_node, tf_buffer_)
    , agent1_(cfg.robot2_ns, cfg.robot2_base_frame, cfg.robot2_home, cfg.map_frame,
              cfg.detector_node, tf_buffer_)
  {
    exploration_.configure(cfg.explore_step, cfg.explore_clearance);

    map_sub_ = nh_.subscribe(cfg_.map_topic, 1, &CtfCoordinator::onMap, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("ctf/markers", 1, true);
    robot1_trail_pub_ = nh_.advertise<nav_msgs::Path>("/ctf/robot1_trail", 1, true);
    robot2_trail_pub_ = nh_.advertise<nav_msgs::Path>("/ctf/robot2_trail", 1, true);
    flag_captured_pub_ = nh_.advertise<std_msgs::Bool>("/ctf/flag_captured", 1, true);
    flag_captured_alias_pub_ = nh_.advertise<std_msgs::Bool>("/flag_captured", 1, true);
    flag_carrier_pub_ = nh_.advertise<std_msgs::String>("/ctf/flag_carrier", 1, true);
    game_state_pub_ = nh_.advertise<std_msgs::String>("/ctf/game_state", 1, true);

    publishCaptureState(false);
    publishGameState("search");
    robot1_trail_.header.frame_id = cfg_.map_frame;
    robot2_trail_.header.frame_id = cfg_.map_frame;
    ROS_INFO("CTF READY: map_topic=%s map_frame=%s robots=%s,%s",
             cfg_.map_topic.c_str(), cfg_.map_frame.c_str(),
             cfg_.robot1_ns.c_str(), cfg_.robot2_ns.c_str());
  }

  void run()
  {
    if (!agent0_.waitForMoveBase() || !agent1_.waitForMoveBase())
    {
      return;
    }
    ros::Duration(1.0).sleep();

    const int carrier = runSearchPhase();
    if (carrier < 0)
    {
      ROS_WARN("Search ended without capture");
      return;
    }
    runChasePhase(carrier);
    ROS_INFO("Game finished.");
  }

private:
  static constexpr size_t kNumAgents = 2;

  ctf_navigation::game::RobotAgent& agent(size_t index)
  {
    return (index == 0) ? agent0_ : agent1_;
  }

  const ctf_navigation::game::RobotAgent& agent(size_t index) const
  {
    return (index == 0) ? agent0_ : agent1_;
  }

  struct ApproachGoal
  {
    double x;
    double y;
    double yaw;
  };

  ApproachGoal makeFlagApproachGoal(const ctf_navigation::tf_helper::Pose2D& robot,
                                    double flag_x,
                                    double flag_y) const
  {
    const double dx = flag_x - robot.x;
    const double dy = flag_y - robot.y;
    const double dist = std::hypot(dx, dy);

    ApproachGoal goal;
    goal.yaw = std::atan2(dy, dx);
    if (dist > cfg_.flag_standoff_distance && dist > 1e-3)
    {
      const double ux = dx / dist;
      const double uy = dy / dist;
      goal.x = flag_x - ux * cfg_.flag_standoff_distance;
      goal.y = flag_y - uy * cfg_.flag_standoff_distance;
    }
    else
    {
      goal.x = robot.x;
      goal.y = robot.y;
    }
    return goal;
  }

  ApproachGoal makeInterceptionGoal(const ctf_navigation::tf_helper::Pose2D& carrier,
                                    const ctf_navigation::tf_helper::Pose2D& pursuer,
                                    const ctf_navigation::game::HomePose& carrier_home) const
  {
    ApproachGoal goal;

    const double home_dx = carrier_home.x - carrier.x;
    const double home_dy = carrier_home.y - carrier.y;
    const double home_dist = std::hypot(home_dx, home_dy);
    if (!cfg_.intercept_enabled || home_dist < 1e-3)
    {
      goal.x = carrier.x;
      goal.y = carrier.y;
      goal.yaw = std::atan2(carrier.y - pursuer.y, carrier.x - pursuer.x);
      return goal;
    }

    const double robot_distance = ctf_navigation::geometry::distance2D(
        carrier.x, carrier.y, pursuer.x, pursuer.y);
    if (robot_distance <= cfg_.intercept_direct_chase_distance)
    {
      goal.x = carrier.x;
      goal.y = carrier.y;
      goal.yaw = std::atan2(carrier.y - pursuer.y, carrier.x - pursuer.x);
      return goal;
    }

    const double ux = home_dx / home_dist;
    const double uy = home_dy / home_dist;
    const double step = std::max(0.10, cfg_.intercept_step);
    double chosen_s = -1.0;

    for (double s = step; s <= home_dist; s += step)
    {
      const double ix = carrier.x + ux * s;
      const double iy = carrier.y + uy * s;
      const double pursuer_dist =
          ctf_navigation::geometry::distance2D(pursuer.x, pursuer.y, ix, iy);
      if (pursuer_dist + cfg_.intercept_arrival_margin <= s)
      {
        chosen_s = s;
        break;
      }
    }

    if (chosen_s < 0.0)
    {
      chosen_s = std::min(home_dist, std::max(step, cfg_.intercept_min_lead));
    }

    goal.x = carrier.x + ux * chosen_s;
    goal.y = carrier.y + uy * chosen_s;
    goal.yaw = std::atan2(goal.y - pursuer.y, goal.x - pursuer.x);
    return goal;
  }

  void onMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    map_received_ = true;
    const size_t previous_waypoint_count = last_waypoint_count_;
    exploration_.updateFromMap(*msg);
    if (exploration_.ready())
    {
      const size_t count = exploration_.waypoints().size();
      if (count != last_waypoint_count_)
      {
        ROS_INFO("CTF MAP: %zu exploration goals active from %s",
                 count, cfg_.map_topic.c_str());
        last_waypoint_count_ = count;
        if (previous_waypoint_count > 0)
        {
          agent0_.clearTargetWaypoint();
          agent1_.clearTargetWaypoint();
        }
      }
    }
    else
    {
      ROS_WARN_THROTTLE(5.0,
                        "Map received (%ux%u) but no free waypoints "
                        "(try lowering explore_clearance)",
                        msg->info.width, msg->info.height);
    }
  }

  int runSearchPhase()
  {
    ROS_INFO("CTF PHASE 1 SEARCH: explore the map and look for the red flag");
    ros::Rate rate(cfg_.rate_hz);

    const ros::Time map_wait_start = ros::Time::now();
    while (ros::ok() && !exploration_.ready())
    {
      ros::spinOnce();
      if (!map_received_)
      {
        ROS_INFO_THROTTLE(2.0, "Waiting for %s...", cfg_.map_topic.c_str());
      }
      else
      {
        ROS_INFO_THROTTLE(2.0,
                          "Map received; waiting for exploration waypoints...");
      }
      if ((ros::Time::now() - map_wait_start).toSec() > cfg_.map_wait_timeout)
      {
        ROS_ERROR("Timeout waiting for map/exploration. Check: "
                  "rostopic echo %s, map server/map_merge, explore_clearance",
                  cfg_.map_topic.c_str());
        return -1;
      }
      rate.sleep();
    }

    while (ros::ok())
    {
      ros::spinOnce();

      for (size_t i = 0; i < kNumAgents; ++i)
      {
        auto& ag = agent(i);
        const auto pose = ag.poseInMap();
        if (!pose)
        {
          continue;
        }

        const bool fresh = ag.hasFreshFlagEstimate(cfg_.flag_memory_timeout);

        if (fresh)
        {
          const auto est = ag.flagEstimate();
          if (est)
          {
            const double distance_to_flag = ctf_navigation::geometry::distance2D(
                pose->x, pose->y, est->first, est->second);
            if (distance_to_flag <= cfg_.flag_capture_distance)
            {
              ROS_INFO("CTF CAPTURE: %s reached the flag at (%.2f, %.2f), distance=%.2f m",
                       ag.ns().c_str(), est->first, est->second,
                       distance_to_flag);
              publishFlagCarrier(ag.ns());
              publishGameState("flag_captured");
              pauseForCapture();
              return static_cast<int>(i);
            }
          }
        }

        if (fresh)
        {
          if (ag.searchState() != ctf_navigation::game::SearchState::PURSUING_FLAG)
          {
            ROS_INFO("CTF FLAG: %s localized the flag; switching to approach",
                     ag.ns().c_str());
            ag.setSearchState(ctf_navigation::game::SearchState::PURSUING_FLAG);
            ag.clearTargetWaypoint();
            ag.resetFlagGoalThrottle();
          }
          const auto est = ag.flagEstimate();
          if (est && ag.shouldSendFlagGoal(est->first, est->second, 0.3, 1.0))
          {
            const auto goal = makeFlagApproachGoal(*pose, est->first, est->second);
            const double distance_to_flag = ctf_navigation::geometry::distance2D(
                pose->x, pose->y, est->first, est->second);
            ROS_INFO("CTF FLAG: %s approach goal=(%.2f, %.2f) flag=(%.2f, %.2f) "
                     "distance=%.2f m",
                     ag.ns().c_str(), goal.x, goal.y, est->first, est->second,
                     distance_to_flag);
            ag.sendGoal(goal.x, goal.y, goal.yaw);
          }
        }
        else if (ag.searchState() != ctf_navigation::game::SearchState::EXPLORING)
        {
          ROS_INFO("CTF FLAG: %s lost the estimate; returning to exploration",
                   ag.ns().c_str());
          ag.setSearchState(ctf_navigation::game::SearchState::EXPLORING);
        }
      }

      assignExploration();
      publishMarkers();
      rate.sleep();
    }
    return -1;
  }

  void assignExploration()
  {
    const int other_wp0 = agent0_.targetWaypoint();
    const int other_wp1 = agent1_.targetWaypoint();

    for (size_t i = 0; i < kNumAgents; ++i)
    {
      auto& ag = agent(i);
      if (ag.searchState() != ctf_navigation::game::SearchState::EXPLORING)
      {
        continue;
      }

      const auto pose = ag.poseInMap();
      if (!pose)
      {
        continue;
      }

      const int other_target = (i == 0) ? other_wp1 : other_wp0;

      const int wp_idx = ag.targetWaypoint();
      if (wp_idx >= 0 &&
          static_cast<size_t>(wp_idx) < exploration_.waypoints().size())
      {
        const auto& wp = exploration_.waypoints()[static_cast<size_t>(wp_idx)];
        const bool reached = ctf_navigation::geometry::distance2D(
                                 pose->x, pose->y, wp.first, wp.second) <=
                             cfg_.waypoint_reached_dist;
        if (reached || ag.moveBaseTerminal())
        {
          exploration_.markVisited(wp_idx);
          ag.clearTargetWaypoint();
        }
      }

      exploration_.resetVisitedIfComplete();

      if (ag.targetWaypoint() < 0)
      {
        const int idx = exploration_.nearestUnvisited(pose->x, pose->y, other_target);
        if (idx >= 0 &&
            static_cast<size_t>(idx) < exploration_.waypoints().size())
        {
          ag.setTargetWaypoint(idx);
          const auto& wp = exploration_.waypoints()[static_cast<size_t>(idx)];
          ROS_INFO("CTF EXPLORATION: %s -> waypoint %d/%zu at (%.2f, %.2f)",
                   ag.ns().c_str(), idx, exploration_.waypoints().size(),
                   wp.first, wp.second);
          ag.sendGoal(wp.first, wp.second, 0.0);
        }
      }
    }
  }

  int runChasePhase(int carrier_index)
  {
    if (carrier_index < 0 || carrier_index >= static_cast<int>(kNumAgents))
    {
      ROS_ERROR("Invalid carrier index: %d", carrier_index);
      return -1;
    }
    const int pursuer_index = 1 - carrier_index;
    auto& carrier = agent(carrier_index);
    auto& pursuer = agent(pursuer_index);

    ROS_INFO("CTF PHASE 2 CHASE: %s returns home; %s tries to intercept", carrier.ns().c_str(),
             pursuer.ns().c_str());
    publishGameState("chase");

    ros::Rate rate(cfg_.rate_hz);
    ros::Time last_chase_goal = sendInitialChaseGoals(carrier, pursuer);

    while (ros::ok())
    {
      ros::spinOnce();
      publishCaptureState(true);

      if (carrier.moveBaseSucceeded())
      {
        pursuer.cancelGoals();
        ROS_INFO("CTF RESULT: %s reached base before being caught", carrier.ns().c_str());
        publishGameState("carrier_won");
        return 0;
      }

      const auto cpose = carrier.poseInMap();
      const auto ppose = pursuer.poseInMap();
      if (!cpose || !ppose)
      {
        rate.sleep();
        continue;
      }

      const double d = ctf_navigation::geometry::distance2D(
          cpose->x, cpose->y, ppose->x, ppose->y);
      ROS_INFO_THROTTLE(1.0, "Distance %s-%s: %.2f m", carrier.ns().c_str(),
                        pursuer.ns().c_str(), d);

      if (d <= cfg_.catch_distance)
      {
        carrier.cancelGoals();
        pursuer.cancelGoals();
        ROS_INFO("CTF RESULT: %s caught %s", pursuer.ns().c_str(), carrier.ns().c_str());
        publishGameState("pursuer_won");
        return 0;
      }

      if ((ros::Time::now() - last_chase_goal).toSec() >= cfg_.chase_goal_period)
      {
        const auto intercept = makeInterceptionGoal(*cpose, *ppose, carrier.home());
        pursuer.sendGoal(intercept.x, intercept.y, intercept.yaw);
        ROS_INFO_THROTTLE(1.0,
                          "CTF CHASE: %s intercept goal=(%.2f, %.2f) on %s route home",
                          pursuer.ns().c_str(), intercept.x, intercept.y,
                          carrier.ns().c_str());
        last_chase_goal = ros::Time::now();
      }

      publishMarkers();
      rate.sleep();
    }
    return -1;
  }

  void pauseForCapture()
  {
    agent0_.cancelGoals();
    agent1_.cancelGoals();

    publishCaptureState(true);
    ROS_INFO("CTF EVENT: published /ctf/flag_captured and /flag_captured = true");
    ROS_INFO("CTF PAUSE: flag captured; both robots paused for %.1f s", cfg_.capture_pause_sec);

    const ros::Time end = ros::Time::now() + ros::Duration(cfg_.capture_pause_sec);
    ros::Rate rate(cfg_.rate_hz);
    while (ros::ok() && ros::Time::now() < end)
    {
      ros::spinOnce();
      publishCaptureState(true);
      publishMarkers();
      rate.sleep();
    }
  }

  void publishCaptureState(bool captured)
  {
    std_msgs::Bool msg;
    msg.data = captured;
    flag_captured_pub_.publish(msg);
    flag_captured_alias_pub_.publish(msg);
  }

  void publishFlagCarrier(const std::string& carrier_ns)
  {
    std_msgs::String msg;
    msg.data = carrier_ns;
    flag_carrier_pub_.publish(msg);
  }

  void publishGameState(const std::string& state)
  {
    std_msgs::String msg;
    msg.data = state;
    game_state_pub_.publish(msg);
  }

  ros::Time sendInitialChaseGoals(ctf_navigation::game::RobotAgent& carrier,
                                  ctf_navigation::game::RobotAgent& pursuer)
  {
    const ros::Time wait_until = ros::Time::now() + ros::Duration(3.0);
    boost::optional<ctf_navigation::tf_helper::Pose2D> cpose;
    boost::optional<ctf_navigation::tf_helper::Pose2D> ppose;
    ros::Rate rate(cfg_.rate_hz);

    while (ros::ok() && ros::Time::now() < wait_until)
    {
      ros::spinOnce();
      cpose = carrier.poseInMap();
      ppose = pursuer.poseInMap();
      if (cpose && ppose)
      {
        break;
      }
      rate.sleep();
    }

    carrier.sendGoal(carrier.home().x, carrier.home().y, carrier.home().yaw);

    if (cpose && ppose)
    {
      const auto intercept = makeInterceptionGoal(*cpose, *ppose, carrier.home());
      pursuer.sendGoal(intercept.x, intercept.y, intercept.yaw);
      ROS_INFO("CTF CHASE START: %s -> base, %s -> intercept (%.2f, %.2f)",
               carrier.ns().c_str(), pursuer.ns().c_str(),
               intercept.x, intercept.y);
      return ros::Time::now();
    }

    ROS_WARN("Chase started without initial pursuer goal: missing TF pose");
    return ros::Time(0);
  }

  void publishMarkers()
  {
    visualization_msgs::MarkerArray arr;
    int id = 0;

    for (size_t i = 0; i < kNumAgents; ++i)
    {
      const auto& h = agent(i).home();
      arr.markers.push_back(ctf_navigation::markers::makeCylinder(
          cfg_.map_frame, "base", id++, h.x, h.y, 0.45, 0.04,
          (i == 0) ? 0.1f : 1.0f, 0.5f, (i == 0) ? 1.0f : 0.1f, 0.7f));

      const auto est = agent(i).flagEstimate();
      if (est && agent(i).hasFreshFlagEstimate(cfg_.flag_memory_timeout))
      {
        arr.markers.push_back(ctf_navigation::markers::makeSphere(
            cfg_.map_frame, "flag_estimate", id++, est->first, est->second, 0.5,
            0.3, 1.0f, 1.0f, 0.0f, 0.9f));
      }
    }
    marker_pub_.publish(arr);
    publishTrails();
  }

  void publishTrails()
  {
    appendTrailPose(agent0_, robot1_trail_, robot1_trail_pub_);
    appendTrailPose(agent1_, robot2_trail_, robot2_trail_pub_);
  }

  void appendTrailPose(ctf_navigation::game::RobotAgent& ag,
                       nav_msgs::Path& trail,
                       ros::Publisher& pub)
  {
    const auto pose = ag.poseInMap();
    if (!pose)
    {
      return;
    }

    const ros::Time now = ros::Time::now();
    if (!trail.poses.empty())
    {
      const auto& last = trail.poses.back().pose.position;
      if (ctf_navigation::geometry::distance2D(last.x, last.y, pose->x, pose->y) <
          cfg_.trail_min_distance)
      {
        trail.header.stamp = now;
        pub.publish(trail);
        return;
      }
    }

    geometry_msgs::PoseStamped stamped;
    stamped.header.frame_id = cfg_.map_frame;
    stamped.header.stamp = now;
    stamped.pose.position.x = pose->x;
    stamped.pose.position.y = pose->y;
    stamped.pose.orientation = ctf_navigation::geometry::yawToQuaternion(pose->yaw);

    trail.header.frame_id = cfg_.map_frame;
    trail.header.stamp = now;
    trail.poses.push_back(stamped);
    if (cfg_.trail_max_poses > 0 &&
        trail.poses.size() > static_cast<size_t>(cfg_.trail_max_poses))
    {
      trail.poses.erase(trail.poses.begin());
    }
    pub.publish(trail);
  }

  CoordinatorConfig cfg_;
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ctf_navigation::game::ExplorationPlanner exploration_;
  ctf_navigation::game::RobotAgent agent0_;
  ctf_navigation::game::RobotAgent agent1_;

  bool map_received_ = false;
  size_t last_waypoint_count_ = 0;

  ros::Subscriber map_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher robot1_trail_pub_;
  ros::Publisher robot2_trail_pub_;
  ros::Publisher flag_captured_pub_;
  ros::Publisher flag_captured_alias_pub_;
  ros::Publisher flag_carrier_pub_;
  ros::Publisher game_state_pub_;
  nav_msgs::Path robot1_trail_;
  nav_msgs::Path robot2_trail_;
};

CoordinatorConfig loadConfig(ros::NodeHandle& pnh)
{
  CoordinatorConfig cfg;
  pnh.param("map_frame", cfg.map_frame, cfg.map_frame);
  pnh.param("map_topic", cfg.map_topic, cfg.map_topic);
  pnh.param("detector_node", cfg.detector_node, cfg.detector_node);
  pnh.param("rate_hz", cfg.rate_hz, cfg.rate_hz);
  pnh.param("flag_capture_distance", cfg.flag_capture_distance,
            cfg.flag_capture_distance);
  pnh.param("flag_standoff_distance", cfg.flag_standoff_distance,
            cfg.flag_standoff_distance);
  pnh.param("capture_pause_sec", cfg.capture_pause_sec, cfg.capture_pause_sec);
  pnh.param("catch_distance", cfg.catch_distance, cfg.catch_distance);
  pnh.param("flag_memory_timeout", cfg.flag_memory_timeout, cfg.flag_memory_timeout);
  pnh.param("waypoint_reached_dist", cfg.waypoint_reached_dist,
            cfg.waypoint_reached_dist);
  pnh.param("chase_goal_period", cfg.chase_goal_period, cfg.chase_goal_period);
  pnh.param("explore_step", cfg.explore_step, cfg.explore_step);
  pnh.param("explore_clearance", cfg.explore_clearance, cfg.explore_clearance);
  pnh.param("map_wait_timeout", cfg.map_wait_timeout, cfg.map_wait_timeout);
  pnh.param("trail_min_distance", cfg.trail_min_distance, cfg.trail_min_distance);
  pnh.param("trail_max_poses", cfg.trail_max_poses, cfg.trail_max_poses);
  pnh.param("intercept_enabled", cfg.intercept_enabled, cfg.intercept_enabled);
  pnh.param("intercept_direct_chase_distance", cfg.intercept_direct_chase_distance,
            cfg.intercept_direct_chase_distance);
  pnh.param("intercept_arrival_margin", cfg.intercept_arrival_margin,
            cfg.intercept_arrival_margin);
  pnh.param("intercept_step", cfg.intercept_step, cfg.intercept_step);
  pnh.param("intercept_min_lead", cfg.intercept_min_lead, cfg.intercept_min_lead);
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
  ros::init(argc, argv, "ctf_coordinator");
  ros::NodeHandle pnh("~");
  const CoordinatorConfig cfg = loadConfig(pnh);
  CtfCoordinator coordinator(cfg);
  coordinator.run();
  return 0;
}
