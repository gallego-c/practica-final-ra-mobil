/**
 * @file ctf_coordinator_node.cpp
 * @brief Coordinador del juego CTF: exploración, visión, captura y persecución.
 */
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include <boost/optional.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
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

// #region agent log
std::string debugLogPath()
{
  static const std::string path = ros::package::getPath("ctf_navigation") + "/debug-36a89d.log";
  return path;
}

void agentDebugLog(const char* location,
                   const char* message,
                   const char* hypothesis_id,
                   const std::string& data_json)
{
  const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
  const std::string line = std::string("{\"sessionId\":\"36a89d\",\"location\":\"") + location +
                           "\",\"message\":\"" + message + "\",\"hypothesisId\":\"" +
                           hypothesis_id + "\",\"data\":" + data_json +
                           ",\"timestamp\":" + std::to_string(timestamp) + "}\n";

  std::ofstream log(debugLogPath(), std::ios::app);
  if (log)
  {
    log << line;
  }
}
// #endregion

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
  double intercept_direct_chase_distance = 1.25;
  double intercept_arrival_margin = 0.30;
  double intercept_step = 0.25;
  double intercept_min_lead = 1.00;
  double intercept_carrier_move_threshold = 1.25;
  double carrier_home_resend_cooldown = 3.0;
  double direct_chase_refresh_sec = 2.0;
  double direct_chase_carrier_move = 0.25;

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
          if (agent0_.searchState() == ctf_navigation::game::SearchState::EXPLORING)
          {
            agent0_.clearTargetWaypoint();
          }
          if (agent1_.searchState() == ctf_navigation::game::SearchState::EXPLORING)
          {
            agent1_.clearTargetWaypoint();
          }
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

  void updateSearchFlagStates()
  {
    for (size_t i = 0; i < kNumAgents; ++i)
    {
      auto& ag = agent(i);
      if (ag.hasFreshFlagEstimate(cfg_.flag_memory_timeout))
      {
        if (ag.searchState() != ctf_navigation::game::SearchState::PURSUING_FLAG)
        {
          ROS_INFO("CTF FLAG: %s localized the flag; switching to approach", ag.ns().c_str());
          ag.setSearchState(ctf_navigation::game::SearchState::PURSUING_FLAG);
          ag.clearTargetWaypoint();
          ag.resetFlagGoalThrottle();
        }
      }
      else if (ag.searchState() != ctf_navigation::game::SearchState::EXPLORING)
      {
        ROS_INFO("CTF FLAG: %s lost the estimate; returning to exploration", ag.ns().c_str());
        ag.setSearchState(ctf_navigation::game::SearchState::EXPLORING);
        ag.resetFlagGoalThrottle();
        ag.cancelGoals();
      }
    }
  }

  boost::optional<int> findFirstCaptureCandidate() const
  {
    for (size_t i = 0; i < kNumAgents; ++i)
    {
      const auto& ag = agent(i);
      const auto pose = ag.poseInMap();
      if (!pose || !ag.hasFreshFlagEstimate(cfg_.flag_memory_timeout))
      {
        continue;
      }

      const auto est = ag.flagEstimate();
      if (!est)
      {
        continue;
      }

      const double distance_to_flag = ctf_navigation::geometry::distance2D(
          pose->x, pose->y, est->first, est->second);
      if (distance_to_flag <= cfg_.flag_capture_distance)
      {
        return static_cast<int>(i);
      }
    }

    return boost::none;
  }

  void sendFlagApproachGoals()
  {
    for (size_t i = 0; i < kNumAgents; ++i)
    {
      auto& ag = agent(i);
      if (ag.searchState() != ctf_navigation::game::SearchState::PURSUING_FLAG)
      {
        continue;
      }

      const auto pose = ag.poseInMap();
      if (!pose || !ag.hasFreshFlagEstimate(cfg_.flag_memory_timeout))
      {
        continue;
      }

      const auto est = ag.flagEstimate();
      if (!est)
      {
        continue;
      }

      if (ag.shouldSendFlagGoal(est->first, est->second, 0.3, 1.0))
      {
        const auto goal = makeFlagApproachGoal(*pose, est->first, est->second);
        const double distance_to_flag = ctf_navigation::geometry::distance2D(
            pose->x, pose->y, est->first, est->second);
        ROS_INFO("CTF FLAG: %s approach goal=(%.2f, %.2f) flag=(%.2f, %.2f) distance=%.2f m",
                 ag.ns().c_str(), goal.x, goal.y, est->first, est->second, distance_to_flag);
        ag.sendGoal(goal.x, goal.y, goal.yaw);
      }
    }
  }

  void prepareAgentsForChase(int carrier_index)
  {
    for (size_t i = 0; i < kNumAgents; ++i)
    {
      agent(i).resetForPhaseTransition();
    }

    publishFlagCarrier(agent(static_cast<size_t>(carrier_index)).ns());
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

      updateSearchFlagStates();

      const auto capture_candidate = findFirstCaptureCandidate();
      if (capture_candidate)
      {
        const int carrier_index = *capture_candidate;
        const auto& ag = agent(static_cast<size_t>(carrier_index));
        const auto pose = ag.poseInMap();
        const auto est = ag.flagEstimate();
        const double distance_to_flag =
            (pose && est)
                ? ctf_navigation::geometry::distance2D(pose->x, pose->y, est->first, est->second)
                : -1.0;

        ROS_INFO("CTF CAPTURE: %s reached the flag, distance=%.2f m", ag.ns().c_str(),
                 distance_to_flag);
        publishFlagCarrier(ag.ns());
        publishGameState("flag_captured");
        pauseForCapture();
        return carrier_index;
      }

      sendFlagApproachGoals();

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

    prepareAgentsForChase(carrier_index);

    ROS_INFO("CTF PHASE 2 CHASE: %s returns home; %s tries to intercept", carrier.ns().c_str(),
             pursuer.ns().c_str());
    publishGameState("chase");

    ros::Rate rate(cfg_.rate_hz);

    bool has_intercept_goal = false;
    double last_intercept_x = 0.0;
    double last_intercept_y = 0.0;
    double intercept_anchor_cx = 0.0;
    double intercept_anchor_cy = 0.0;
    bool pursuer_was_succeeded = false;
    ros::Time last_carrier_home_resend = ros::Time(0);
    ros::Time last_direct_chase_sent = ros::Time(0);
    ros::Time last_carrier_debug_log = ros::Time(0);
    double last_logged_home_dist = -1.0;
    int carrier_stuck_ticks = 0;

    const auto sendPursuerDirectChase = [&](const ctf_navigation::tf_helper::Pose2D& cpose,
                                            const ctf_navigation::tf_helper::Pose2D& ppose) {
      const double yaw =
          std::atan2(cpose.y - ppose.y, cpose.x - ppose.x);
      pursuer.sendGoal(cpose.x, cpose.y, yaw);
      has_intercept_goal = true;
      intercept_anchor_cx = cpose.x;
      intercept_anchor_cy = cpose.y;
      pursuer_was_succeeded = false;
      last_direct_chase_sent = ros::Time::now();
    };

    const auto sendPursuerIntercept = [&](const ctf_navigation::tf_helper::Pose2D& cpose,
                                          const ctf_navigation::tf_helper::Pose2D& ppose) {
      const auto intercept = makeInterceptionGoal(cpose, ppose, carrier.home());
      pursuer.sendGoal(intercept.x, intercept.y, intercept.yaw);
      has_intercept_goal = true;
      last_intercept_x = intercept.x;
      last_intercept_y = intercept.y;
      intercept_anchor_cx = cpose.x;
      intercept_anchor_cy = cpose.y;
      pursuer_was_succeeded = false;
    };

    const auto maybeUpdatePursuerIntercept =
        [&](const ctf_navigation::tf_helper::Pose2D& cpose,
            const ctf_navigation::tf_helper::Pose2D& ppose) {
          const double carrier_move = ctf_navigation::geometry::distance2D(
              intercept_anchor_cx, intercept_anchor_cy, cpose.x, cpose.y);
          if (!has_intercept_goal ||
              carrier_move >= cfg_.intercept_carrier_move_threshold)
          {
            sendPursuerIntercept(cpose, ppose);
          }
        };

    carrier.cancelGoals();
    pursuer.cancelGoals();

    const ros::Time settle_until = ros::Time::now() + ros::Duration(0.5);
    while (ros::ok() && ros::Time::now() < settle_until)
    {
      ros::spinOnce();
      rate.sleep();
    }

    const ros::Time tf_wait_until = ros::Time::now() + ros::Duration(3.0);
    boost::optional<ctf_navigation::tf_helper::Pose2D> initial_cpose;
    boost::optional<ctf_navigation::tf_helper::Pose2D> initial_ppose;
    while (ros::ok() && ros::Time::now() < tf_wait_until)
    {
      ros::spinOnce();
      initial_cpose = carrier.poseInMap();
      initial_ppose = pursuer.poseInMap();
      if (initial_cpose && initial_ppose)
      {
        break;
      }
      rate.sleep();
    }

    if (initial_cpose && initial_ppose)
    {
      carrier.sendGoal(carrier.home().x, carrier.home().y, carrier.home().yaw);
      const double start_home_dist = ctf_navigation::geometry::distance2D(
          initial_cpose->x, initial_cpose->y, carrier.home().x, carrier.home().y);
      // #region agent log
      {
        std::ostringstream data;
        data << "{\"carrierNs\":\"" << carrier.ns() << "\",\"homeX\":" << carrier.home().x
             << ",\"homeY\":" << carrier.home().y << ",\"carrierX\":" << initial_cpose->x
             << ",\"carrierY\":" << initial_cpose->y << ",\"homeDist\":" << start_home_dist
             << ",\"carrierMbState\":\"" << carrier.moveBaseStateText() << "\"}";
        agentDebugLog("ctf_coordinator_node.cpp:runChasePhase",
                      "carrier home goal sent at chase start", "C", data.str());
      }
      // #endregion
      ROS_INFO("CTF CHASE START: %s -> base (%.2f, %.2f)",
               carrier.ns().c_str(), carrier.home().x, carrier.home().y);

      const ros::Time pursuer_start_after =
          ros::Time::now() + ros::Duration(1.0);
      while (ros::ok() && ros::Time::now() < pursuer_start_after)
      {
        ros::spinOnce();
        rate.sleep();
      }

      initial_cpose = carrier.poseInMap();
      initial_ppose = pursuer.poseInMap();
      if (initial_cpose && initial_ppose)
      {
        sendPursuerIntercept(*initial_cpose, *initial_ppose);
        ROS_INFO("CTF CHASE START: %s -> intercept (%.2f, %.2f)",
                 pursuer.ns().c_str(), last_intercept_x, last_intercept_y);
      }
    }
    else
    {
      // #region agent log
      {
        std::ostringstream data;
        data << "{\"carrierNs\":\"" << carrier.ns() << "\",\"hasCarrierPose\":"
             << (initial_cpose.has_value() ? "true" : "false") << ",\"hasPursuerPose\":"
             << (initial_ppose.has_value() ? "true" : "false") << "}";
        agentDebugLog("ctf_coordinator_node.cpp:runChasePhase",
                      "chase start missing TF pose", "C", data.str());
      }
      // #endregion
      ROS_WARN("Chase started without initial pursuer goal: missing TF pose");
    }

    while (ros::ok())
    {
      ros::spinOnce();
      publishCaptureState(true);

      const auto cpose = carrier.poseInMap();
      const auto ppose = pursuer.poseInMap();
      if (!cpose || !ppose)
      {
        rate.sleep();
        continue;
      }

      const double d = ctf_navigation::geometry::distance2D(
          cpose->x, cpose->y, ppose->x, ppose->y);
      const double carrier_home_dist = ctf_navigation::geometry::distance2D(
          cpose->x, cpose->y, carrier.home().x, carrier.home().y);
      ROS_INFO_THROTTLE(1.0, "Distance %s-%s: %.2f m; %s home dist: %.2f m",
                        carrier.ns().c_str(), pursuer.ns().c_str(), d,
                        carrier.ns().c_str(), carrier_home_dist);

      if (d <= cfg_.catch_distance)
      {
        carrier.cancelGoals();
        pursuer.cancelGoals();
        ROS_INFO("CTF RESULT: %s caught %s", pursuer.ns().c_str(), carrier.ns().c_str());
        publishGameState("pursuer_won");
        return 0;
      }

      if (carrier_home_dist <= cfg_.waypoint_reached_dist)
      {
        pursuer.cancelGoals();
        ROS_INFO("CTF RESULT: %s reached base before being caught", carrier.ns().c_str());
        publishGameState("carrier_won");
        return 0;
      }

      if (carrier.moveBaseTerminal() && !carrier.moveBaseSucceeded())
      {
        const ros::Time now = ros::Time::now();
        const double cooldown_left =
            cfg_.carrier_home_resend_cooldown - (now - last_carrier_home_resend).toSec();
        if ((now - last_carrier_home_resend).toSec() >= cfg_.carrier_home_resend_cooldown)
        {
          carrier.sendGoal(carrier.home().x, carrier.home().y, carrier.home().yaw);
          last_carrier_home_resend = now;
          // #region agent log
          {
            std::ostringstream data;
            data << "{\"carrierNs\":\"" << carrier.ns() << "\",\"homeDist\":" << carrier_home_dist
                 << ",\"carrierMbState\":\"" << carrier.moveBaseStateText() << "\"}";
            agentDebugLog("ctf_coordinator_node.cpp:runChasePhase",
                          "carrier home goal re-sent after failure", "B", data.str());
          }
          // #endregion
          ROS_WARN("CTF CHASE: %s home goal re-sent after navigation failure",
                   carrier.ns().c_str());
        }
        else
        {
          // #region agent log
          {
            std::ostringstream data;
            data << "{\"carrierNs\":\"" << carrier.ns() << "\",\"homeDist\":" << carrier_home_dist
                 << ",\"cooldownLeftSec\":" << cooldown_left << ",\"carrierMbState\":\""
                 << carrier.moveBaseStateText() << "\"}";
            agentDebugLog("ctf_coordinator_node.cpp:runChasePhase",
                          "carrier resend blocked by cooldown", "B", data.str());
          }
          // #endregion
        }
      }

      if ((ros::Time::now() - last_carrier_debug_log).toSec() >= 2.0)
      {
        if (last_logged_home_dist >= 0.0 &&
            std::abs(carrier_home_dist - last_logged_home_dist) < 0.05)
        {
          ++carrier_stuck_ticks;
        }
        else
        {
          carrier_stuck_ticks = 0;
        }
        // #region agent log
        {
          std::ostringstream data;
          data << "{\"carrierNs\":\"" << carrier.ns() << "\",\"homeDist\":" << carrier_home_dist
               << ",\"robotDistance\":" << d << ",\"carrierMbState\":\""
               << carrier.moveBaseStateText() << "\",\"pursuerMbState\":\""
               << pursuer.moveBaseStateText() << "\",\"directChase\":"
               << (d <= cfg_.intercept_direct_chase_distance ? "true" : "false")
               << ",\"stuckTicks\":" << carrier_stuck_ticks << "}";
          agentDebugLog("ctf_coordinator_node.cpp:runChasePhase", "carrier chase tick", "A",
                        data.str());
        }
        // #endregion
        last_logged_home_dist = carrier_home_dist;
        last_carrier_debug_log = ros::Time::now();
      }

      const bool direct_chase = d <= cfg_.intercept_direct_chase_distance;
      if (direct_chase)
      {
        const double carrier_move = ctf_navigation::geometry::distance2D(
            intercept_anchor_cx, intercept_anchor_cy, cpose->x, cpose->y);
        const ros::Time now = ros::Time::now();
        const bool pursuer_failed =
            pursuer.moveBaseTerminal() && !pursuer.moveBaseSucceeded();
        const bool refresh_direct =
            !has_intercept_goal || carrier_move >= cfg_.direct_chase_carrier_move ||
            pursuer_failed ||
            (pursuer.moveBaseSucceeded() && !pursuer_was_succeeded) ||
            (pursuer.moveBaseSucceeded() &&
             (now - last_direct_chase_sent).toSec() >= cfg_.direct_chase_refresh_sec);
        if (refresh_direct)
        {
          sendPursuerDirectChase(*cpose, *ppose);
        }
        if (pursuer.moveBaseSucceeded())
        {
          pursuer_was_succeeded = true;
        }
        else
        {
          pursuer_was_succeeded = false;
        }
      }
      else if (pursuer.moveBaseSucceeded())
      {
        if (!pursuer_was_succeeded)
        {
          sendPursuerIntercept(*cpose, *ppose);
        }
        pursuer_was_succeeded = true;
      }
      else
      {
        pursuer_was_succeeded = false;
        if (pursuer.moveBaseTerminal())
        {
          sendPursuerIntercept(*cpose, *ppose);
        }
        else
        {
          maybeUpdatePursuerIntercept(*cpose, *ppose);
        }
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
  pnh.param("intercept_carrier_move_threshold", cfg.intercept_carrier_move_threshold,
            cfg.intercept_carrier_move_threshold);
  pnh.param("carrier_home_resend_cooldown", cfg.carrier_home_resend_cooldown,
            cfg.carrier_home_resend_cooldown);
  pnh.param("direct_chase_refresh_sec", cfg.direct_chase_refresh_sec,
            cfg.direct_chase_refresh_sec);
  pnh.param("direct_chase_carrier_move", cfg.direct_chase_carrier_move,
            cfg.direct_chase_carrier_move);
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
