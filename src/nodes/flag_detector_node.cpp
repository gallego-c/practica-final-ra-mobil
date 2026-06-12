/**
 * @file flag_detector_node.cpp
 * @brief Detección de la bandera por cámara (HSV rojo) + posición con LIDAR y TF.
 *
 * Publica (en el namespace del robot):
 *   ~/flag_found    (std_msgs/Bool)
 *   ~/flag_estimate (geometry_msgs/PoseStamped, frame map)
 *   ~/debug_image   (sensor_msgs/Image, opcional)
 */
#include <cstdio>
#include <mutex>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ctf_navigation/common/geometry.hpp"
#include "ctf_navigation/common/tf_helper.hpp"
#include "ctf_navigation/vision/laser_utils.hpp"
#include "ctf_navigation/vision/red_flag_detector.hpp"

namespace
{

struct FlagDetectorConfig
{
  std::string camera_topic = "camera/rgb/image_raw";
  std::string compressed_camera_topic = "camera/rgb/image_raw/compressed";
  std::string scan_topic = "scan";
  std::string base_frame = "base_footprint";
  std::string map_frame = "map";
  double horizontal_fov = 1.085595;
  double max_detection_distance = 5.0;
  double range_window = 0.10;
  bool publish_debug = true;
  ctf_navigation::vision::RedDetectorConfig red;
};

class FlagDetectorNode
{
public:
  explicit FlagDetectorNode(const FlagDetectorConfig& cfg)
    : cfg_(cfg)
    , tf_listener_(tf_buffer_)
  {
    // Publicaciones en ~ (/robotN/flag_detector/...).
    ros::NodeHandle pnh("~");
    // Sensores en el namespace del robot (/robotN/scan, /robotN/camera/...).
    ros::NodeHandle robot_nh;

    found_pub_ = pnh.advertise<std_msgs::Bool>("flag_found", 1);
    estimate_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("flag_estimate", 1);
    if (cfg_.publish_debug)
    {
      debug_pub_ = pnh.advertise<sensor_msgs::Image>("debug_image", 1);
    }

    scan_sub_ = robot_nh.subscribe(cfg_.scan_topic, 1, &FlagDetectorNode::onScan, this);
    image_sub_ =
        robot_nh.subscribe(cfg_.camera_topic, 1, &FlagDetectorNode::onImage, this);
    compressed_image_sub_ = robot_nh.subscribe(
        cfg_.compressed_camera_topic, 1, &FlagDetectorNode::onCompressedImage, this);

    ROS_INFO("flag_detector: camera=%s compressed=%s scan=%s base=%s",
             robot_nh.resolveName(cfg_.camera_topic).c_str(),
             robot_nh.resolveName(cfg_.compressed_camera_topic).c_str(),
             robot_nh.resolveName(cfg_.scan_topic).c_str(),
             cfg_.base_frame.c_str());
  }

  bool hasReceivedImages() const { return images_received_; }
  const std::string& cameraTopic() const { return cfg_.camera_topic; }
  const std::string& compressedCameraTopic() const
  {
    return cfg_.compressed_camera_topic;
  }

private:
  void onScan(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    last_scan_ = *msg;
  }

  void onImage(const sensor_msgs::Image::ConstPtr& msg)
  {
    images_received_ = true;

    cv::Mat bgr;
    try
    {
      if (msg->encoding == sensor_msgs::image_encodings::MONO8 ||
          msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1)
      {
        const cv_bridge::CvImageConstPtr gray =
            cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        cv::cvtColor(gray->image, bgr, cv::COLOR_GRAY2BGR);
      }
      else if (msg->encoding == sensor_msgs::image_encodings::RGB8)
      {
        const cv_bridge::CvImageConstPtr rgb =
            cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        cv::cvtColor(rgb->image, bgr, cv::COLOR_RGB2BGR);
      }
      else if (msg->encoding == sensor_msgs::image_encodings::RGBA8)
      {
        const cv_bridge::CvImageConstPtr rgba =
            cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGBA8);
        cv::cvtColor(rgba->image, bgr, cv::COLOR_RGBA2BGR);
      }
      else if (msg->encoding == sensor_msgs::image_encodings::BGRA8)
      {
        const cv_bridge::CvImageConstPtr bgra =
            cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);
        cv::cvtColor(bgra->image, bgr, cv::COLOR_BGRA2BGR);
      }
      else
      {
        const cv_bridge::CvImageConstPtr cv_ptr =
            cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        bgr = cv_ptr->image;
      }
    }
    catch (const cv_bridge::Exception& ex)
    {
      ROS_WARN_THROTTLE(5.0, "cv_bridge (%s): %s", msg->encoding.c_str(), ex.what());
      return;
    }

    processBgrFrame(bgr, msg->header.stamp);
  }

  void onCompressedImage(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {
    images_received_ = true;

    const cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8UC1,
                          const_cast<unsigned char*>(msg->data.data()));
    const cv::Mat bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (bgr.empty())
    {
      ROS_WARN_THROTTLE(5.0, "Could not decode compressed image (%s)",
                        msg->format.c_str());
      return;
    }

    processBgrFrame(bgr, msg->header.stamp);
  }

  void processBgrFrame(const cv::Mat& bgr, const ros::Time& stamp)
  {
    if (!stamp.isZero() && (ros::Time::now() - stamp).toSec() > 1.0)
    {
      ROS_WARN_THROTTLE(2.0, "Stale image received (age = %.2f s), ignoring", (ros::Time::now() - stamp).toSec());
      return;
    }

    const auto det = ctf_navigation::vision::detectRedBlob(bgr, cfg_.red);

    std_msgs::Bool found_msg;
    found_msg.data = det.found;
    found_pub_.publish(found_msg);

    boost::optional<geometry_msgs::PoseStamped> estimate;
    if (det.found)
    {
      const double bearing = ctf_navigation::vision::bearingFromCentroid(
          det.centroid_x, bgr.cols, cfg_.horizontal_fov);
      ROS_DEBUG_THROTTLE(1.0, "Red flag candidate: area=%.0f cx=%.0f bearing=%.2f rad",
                        det.area, det.centroid_x, bearing);
      estimate = estimateFlagPose(bearing, stamp);
      if (estimate)
      {
        estimate_pub_.publish(*estimate);
      }
      else
      {
        ROS_WARN_THROTTLE(3.0,
                          "Red blob seen (area=%.0f) but no LIDAR/TF estimate",
                          det.area);
      }
    }

    if (cfg_.publish_debug)
    {
      publishDebug(bgr, det, estimate);
    }
  }

  boost::optional<geometry_msgs::PoseStamped> estimateFlagPose(double bearing, const ros::Time& stamp)
  {
    sensor_msgs::LaserScan scan;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (last_scan_.ranges.empty())
      {
        ROS_WARN_THROTTLE(5.0, "Flag visible but no LaserScan yet");
        return boost::none;
      }
      if (!stamp.isZero() && !last_scan_.header.stamp.isZero())
      {
        double scan_age = std::abs((last_scan_.header.stamp - stamp).toSec());
        if (scan_age > 1.0)
        {
          ROS_WARN_THROTTLE(2.0, "LaserScan and image are unsynchronized (diff = %.2f s)", scan_age);
          return boost::none;
        }
      }
      scan = last_scan_;
    }

    // Only use LIDAR hits near the camera bearing. A FOV-wide minimum range
    // often picks a nearby wall and places the flag unrealistically close.
    const boost::optional<double> range = ctf_navigation::vision::minRangeAtBearing(
        scan, bearing, cfg_.range_window);
    if (!range || *range > cfg_.max_detection_distance)
    {
      return boost::none;
    }

    geometry_msgs::PoseStamped pose_base;
    pose_base.header.frame_id = cfg_.base_frame;
    pose_base.header.stamp = stamp;
    pose_base.pose.position.x = *range * std::cos(bearing);
    pose_base.pose.position.y = *range * std::sin(bearing);
    pose_base.pose.orientation.w = 1.0;

    auto pose_map = ctf_navigation::tf_helper::transformPose(
        tf_buffer_, pose_base, cfg_.map_frame, 0.2);
    if (pose_map)
    {
      pose_map->pose.orientation.w = 1.0;
    }
    return pose_map;
  }

  void publishDebug(const cv::Mat& frame,
                    const ctf_navigation::vision::RedDetection& det,
                    const boost::optional<geometry_msgs::PoseStamped>& estimate)
  {
    cv::Mat dbg = frame.clone();
    if (!det.mask.empty())
    {
      dbg.setTo(cv::Scalar(0, 255, 0), det.mask);
    }
    if (det.found && det.centroid_x >= 0)
    {
      const int cx = static_cast<int>(det.centroid_x);
      cv::line(dbg, cv::Point(cx, 0), cv::Point(cx, dbg.rows), cv::Scalar(255, 0, 0), 2);
      char buf[128];
      if (estimate)
      {
        snprintf(buf, sizeof(buf), "FLAG area=%d @ (%.2f, %.2f)",
                 static_cast<int>(det.area), estimate->pose.position.x,
                 estimate->pose.position.y);
      }
      else
      {
        snprintf(buf, sizeof(buf), "FLAG area=%d", static_cast<int>(det.area));
      }
      const std::string text(buf);
      cv::putText(dbg, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(0, 255, 255), 2);
    }
    else
    {
      char buf[96];
      snprintf(buf, sizeof(buf), "red area=%d min=%d", static_cast<int>(det.area),
               cfg_.red.min_blob_area);
      cv::putText(dbg, buf, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(0, 255, 255), 2);
    }
    try
    {
      const auto out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dbg).toImageMsg();
      debug_pub_.publish(out);
    }
    catch (const cv_bridge::Exception&)
    {
    }
  }

  FlagDetectorConfig cfg_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex scan_mutex_;
  sensor_msgs::LaserScan last_scan_;
  bool images_received_ = false;

  ros::Subscriber scan_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber compressed_image_sub_;
  ros::Publisher found_pub_;
  ros::Publisher estimate_pub_;
  ros::Publisher debug_pub_;
};

FlagDetectorConfig loadConfig(ros::NodeHandle& pnh)
{
  FlagDetectorConfig cfg;
  pnh.param("camera_topic", cfg.camera_topic, cfg.camera_topic);
  pnh.param("compressed_camera_topic", cfg.compressed_camera_topic,
            cfg.compressed_camera_topic);
  pnh.param("scan_topic", cfg.scan_topic, cfg.scan_topic);
  pnh.param("base_frame", cfg.base_frame, cfg.base_frame);
  pnh.param("map_frame", cfg.map_frame, cfg.map_frame);
  pnh.param("horizontal_fov", cfg.horizontal_fov, cfg.horizontal_fov);
  pnh.param("max_detection_distance", cfg.max_detection_distance,
            cfg.max_detection_distance);
  pnh.param("range_window", cfg.range_window, cfg.range_window);
  pnh.param("publish_debug", cfg.publish_debug, cfg.publish_debug);
  pnh.param("min_blob_area", cfg.red.min_blob_area, cfg.red.min_blob_area);
  pnh.param("h_low1", cfg.red.h_low1, cfg.red.h_low1);
  pnh.param("h_high1", cfg.red.h_high1, cfg.red.h_high1);
  pnh.param("h_low2", cfg.red.h_low2, cfg.red.h_low2);
  pnh.param("h_high2", cfg.red.h_high2, cfg.red.h_high2);
  pnh.param("s_min", cfg.red.s_min, cfg.red.s_min);
  pnh.param("v_min", cfg.red.v_min, cfg.red.v_min);
  pnh.param("use_rgb_fallback", cfg.red.use_rgb_fallback,
            cfg.red.use_rgb_fallback);
  pnh.param("rgb_red_min", cfg.red.rgb_red_min, cfg.red.rgb_red_min);
  pnh.param("rgb_red_dominance", cfg.red.rgb_red_dominance,
            cfg.red.rgb_red_dominance);
  return cfg;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flag_detector");
  ros::NodeHandle pnh("~");
  FlagDetectorNode node(loadConfig(pnh));

  const ros::Time startup = ros::Time::now();
  ros::Rate rate(1.0);
  while (ros::ok() && !node.hasReceivedImages() &&
         (ros::Time::now() - startup).toSec() < 30.0)
  {
    ros::spinOnce();
    ROS_WARN_THROTTLE(5.0,
                      "No camera images yet — check camera_topic / "
                      "compressed_camera_topic (rostopic list | grep image)");
    rate.sleep();
  }
  if (!node.hasReceivedImages())
  {
    ROS_ERROR("No camera images after 30 s. Try: rostopic list | grep image");
  }

  ros::spin();
  return 0;
}
