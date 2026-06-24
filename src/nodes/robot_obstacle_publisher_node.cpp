/**
 * @file robot_obstacle_publisher_node.cpp
 * @brief Publica la huella del otro robot como PointCloud2 para evitar colisiones.
 */
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

constexpr double kRobotRadius = 0.22;
constexpr int kNumPoints = 20;
constexpr double kPublishHz = 5.0;

sensor_msgs::PointCloud2 makeFootprintCloud(const std::string& frame_id,
                                            double cx,
                                            double cy)
{
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  cloud.width = kNumPoints;
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(kNumPoints);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (int i = 0; i < kNumPoints; ++i, ++iter_x, ++iter_y, ++iter_z)
  {
    const double angle = 2.0 * M_PI * i / kNumPoints;
    *iter_x = static_cast<float>(cx + kRobotRadius * std::cos(angle));
    *iter_y = static_cast<float>(cy + kRobotRadius * std::sin(angle));
    *iter_z = 0.0f;
  }
  return cloud;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_obstacle_publisher");
  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  auto pub1 = nh.advertise<sensor_msgs::PointCloud2>("/robot1/other_robot_cloud", 1);
  auto pub2 = nh.advertise<sensor_msgs::PointCloud2>("/robot2/other_robot_cloud", 1);

  struct Pair
  {
    std::string source_ns;
    ros::Publisher* pub;
  };
  Pair pairs[] = {{"robot2", &pub1}, {"robot1", &pub2}};

  ros::Rate rate(kPublishHz);
  ROS_INFO("robot_obstacle_publisher started");

  while (ros::ok())
  {
    for (const auto& p : pairs)
    {
      try
      {
        const auto tf = tf_buffer.lookupTransform(
            "map", p.source_ns + "/base_footprint", ros::Time(0), ros::Duration(0.15));
        const double cx = tf.transform.translation.x;
        const double cy = tf.transform.translation.y;
        p.pub->publish(makeFootprintCloud("map", cx, cy));
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE(5.0, "TF failed: %s", ex.what());
      }
    }
    rate.sleep();
  }
  return 0;
}
