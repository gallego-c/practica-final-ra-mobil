// Filtro de seguridad multinivel para los robots del CTF.
//
// Recibe cmd_vel_raw de move_base y lo modifica antes de publicarlo
// en cmd_vel. Aplica tres mecanismos independientes:
//   1. Distancia al otro robot (via TF) -> escalado y stop si se acerca
//   2. LIDAR 360 -> stop si hay obstaculo cerca en el sector frontal
//   3. LIDAR 360 -> "ramp" lateral para reducir velocidad angular cuando
//      hay obstaculos a los lados durante un giro.
//
// La idea es que esta capa actua como red de seguridad sobre el local
// planner. Si los costmaps + planner hacen su trabajo nunca se dispara,
// pero si fallan (drift de TF, scan retrasado, etc.) evitamos un choque
// fisico en simulacion o en el robot real.

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

double clampValue(double value, double lo, double hi)
{
  return std::max(lo, std::min(hi, value));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

class CmdVelSafetyFilter
{
public:
  CmdVelSafetyFilter()
    : private_nh_("~")
    , tf_listener_(tf_buffer_)
    , scan_min_front_(std::numeric_limits<double>::infinity())
    , scan_min_left_(std::numeric_limits<double>::infinity())
    , scan_min_right_(std::numeric_limits<double>::infinity())
    , scan_min_back_(std::numeric_limits<double>::infinity())
    , last_scan_time_(0.0)
  {
    private_nh_.param<std::string>("global_frame", global_frame_, "map");
    private_nh_.param<std::string>("robot_frame", robot_frame_, "base_footprint");
    private_nh_.param<std::string>("other_robot_frame", other_robot_frame_, "");
    private_nh_.param<std::string>("scan_topic", scan_topic_, "scan");

    // Robot-robot
    private_nh_.param("stop_distance", stop_distance_, 0.45);
    private_nh_.param("slow_distance", slow_distance_, 1.00);
    private_nh_.param("min_scale", min_scale_, 0.20);

    // Scan-based
    private_nh_.param("scan_stop_distance", scan_stop_distance_, 0.28);
    private_nh_.param("scan_slow_distance", scan_slow_distance_, 0.60);
    private_nh_.param("scan_front_half_fov", scan_front_half_fov_, 0.785);  // ~45deg

    // Margen de tiempo para considerar el scan "fresco"
    private_nh_.param("scan_max_age", scan_max_age_, 0.5);
    private_nh_.param("enabled", enabled_, true);

    cmd_sub_ = nh_.subscribe("cmd_vel_raw", 1, &CmdVelSafetyFilter::cmdCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &CmdVelSafetyFilter::scanCallback, this);

    ROS_INFO("[%s] safety filter: stop_other=%.2f m slow_other=%.2f m | "
             "scan_stop=%.2f m scan_slow=%.2f m frontHFoV=%.2f rad topic=%s",
             robot_frame_.c_str(), stop_distance_, slow_distance_,
             scan_stop_distance_, scan_slow_distance_, scan_front_half_fov_,
             scan_topic_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    double min_front = std::numeric_limits<double>::infinity();
    double min_left  = std::numeric_limits<double>::infinity();
    double min_right = std::numeric_limits<double>::infinity();
    double min_back  = std::numeric_limits<double>::infinity();

    const double range_min = std::max(static_cast<double>(msg->range_min), 0.05);
    const double range_max = static_cast<double>(msg->range_max);

    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      const double r = static_cast<double>(msg->ranges[i]);
      if (!std::isfinite(r) || r <= range_min || r >= range_max)
      {
        continue;
      }
      const double angle = normalizeAngle(msg->angle_min + i * msg->angle_increment);
      const double abs_a = std::fabs(angle);

      if (abs_a <= scan_front_half_fov_)
      {
        if (r < min_front) min_front = r;
      }
      else if (abs_a >= M_PI - scan_front_half_fov_)
      {
        if (r < min_back) min_back = r;
      }
      else if (angle > 0.0)
      {
        if (r < min_left) min_left = r;
      }
      else
      {
        if (r < min_right) min_right = r;
      }
    }

    std::lock_guard<std::mutex> lk(scan_mutex_);
    scan_min_front_ = min_front;
    scan_min_left_  = min_left;
    scan_min_right_ = min_right;
    scan_min_back_  = min_back;
    last_scan_time_ = ros::Time::now().toSec();
  }

  // Devuelve un factor de escala [0, 1] para la velocidad lineal en funcion
  // del LIDAR. Si el obstaculo frontal esta mas cerca que scan_stop_distance
  // bloqueamos el avance. Devuelve un escalado lineal a medio camino.
  double linearScanScale(double v_linear) const
  {
    if (last_scan_time_ <= 0.0)
    {
      return 1.0;
    }
    const double now = ros::Time::now().toSec();
    if (now - last_scan_time_ > scan_max_age_)
    {
      // Sin lectura reciente, ser conservador
      return 0.5;
    }

    double front;
    {
      std::lock_guard<std::mutex> lk(scan_mutex_);
      front = scan_min_front_;
    }

    // Si vamos hacia atras, no aplicamos el sector frontal
    if (v_linear < 0.0)
    {
      return 1.0;
    }
    if (front <= scan_stop_distance_)
    {
      return 0.0;
    }
    if (front >= scan_slow_distance_)
    {
      return 1.0;
    }
    const double span = std::max(0.01, scan_slow_distance_ - scan_stop_distance_);
    const double scale = (front - scan_stop_distance_) / span;
    return clampValue(min_scale_ + (1.0 - min_scale_) * scale, 0.0, 1.0);
  }

  // En giros muy cerrados con obstaculo lateral, reducir omega para
  // evitar barrer al obstaculo con el chasis.
  double angularScanScale(double omega) const
  {
    if (last_scan_time_ <= 0.0)
    {
      return 1.0;
    }
    double min_side;
    {
      std::lock_guard<std::mutex> lk(scan_mutex_);
      min_side = (omega >= 0.0) ? scan_min_left_ : scan_min_right_;
    }
    if (!std::isfinite(min_side) || min_side >= scan_slow_distance_)
    {
      return 1.0;
    }
    if (min_side <= scan_stop_distance_)
    {
      // Aun permitir un giro lento para escapar
      return min_scale_;
    }
    const double span = std::max(0.01, scan_slow_distance_ - scan_stop_distance_);
    const double scale = (min_side - scan_stop_distance_) / span;
    return clampValue(min_scale_ + (1.0 - min_scale_) * scale, min_scale_, 1.0);
  }

  // Distancia al otro robot via TF (si esta configurado)
  bool otherRobotInfo(double& distance, double& closing_speed,
                      double v_linear_world_x, double v_linear_world_y) const
  {
    if (other_robot_frame_.empty())
    {
      return false;
    }
    geometry_msgs::TransformStamped robot_tf;
    geometry_msgs::TransformStamped other_tf;
    try
    {
      robot_tf = tf_buffer_.lookupTransform(global_frame_, robot_frame_,
                                            ros::Time(0), ros::Duration(0.05));
      other_tf = tf_buffer_.lookupTransform(global_frame_, other_robot_frame_,
                                            ros::Time(0), ros::Duration(0.05));
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(2.0, "[%s] safety filter sin TF: %s",
                        robot_frame_.c_str(), ex.what());
      return false;
    }

    const double dx = other_tf.transform.translation.x - robot_tf.transform.translation.x;
    const double dy = other_tf.transform.translation.y - robot_tf.transform.translation.y;
    distance = std::sqrt(dx * dx + dy * dy);
    if (distance < 1e-3)
    {
      closing_speed = 0.0;
    }
    else
    {
      closing_speed = (v_linear_world_x * dx + v_linear_world_y * dy) / distance;
    }
    return true;
  }

  void cmdCallback(const geometry_msgs::Twist& raw_cmd)
  {
    geometry_msgs::Twist safe_cmd = raw_cmd;
    if (!enabled_)
    {
      cmd_pub_.publish(safe_cmd);
      return;
    }

    // 1) Escalado por LIDAR (sector frontal)
    double scan_lin_scale = linearScanScale(raw_cmd.linear.x);
    double scan_ang_scale = angularScanScale(raw_cmd.angular.z);
    if (scan_lin_scale <= 0.0)
    {
      // No avanzar pero permitir giro reducido para escapar
      safe_cmd.linear.x = 0.0;
      safe_cmd.angular.z = raw_cmd.angular.z * std::max(scan_ang_scale, min_scale_);
      ROS_WARN_THROTTLE(1.0, "[%s] STOP LIDAR (front<%.2f)", robot_frame_.c_str(),
                        scan_stop_distance_);
      cmd_pub_.publish(safe_cmd);
      return;
    }
    safe_cmd.linear.x  = raw_cmd.linear.x  * scan_lin_scale;
    safe_cmd.angular.z = raw_cmd.angular.z * scan_ang_scale;

    // 2) Escalado por otro robot
    geometry_msgs::TransformStamped robot_tf;
    try
    {
      robot_tf = tf_buffer_.lookupTransform(global_frame_, robot_frame_,
                                            ros::Time(0), ros::Duration(0.05));
    }
    catch (const tf2::TransformException&)
    {
      // No tenemos TF para calcular velocidad mundial; publicar lo que ya tenemos
      cmd_pub_.publish(safe_cmd);
      return;
    }

    const double yaw = tf2::getYaw(robot_tf.transform.rotation);
    const double vx_world = safe_cmd.linear.x * std::cos(yaw);
    const double vy_world = safe_cmd.linear.x * std::sin(yaw);

    double other_distance = 0.0;
    double closing_speed = 0.0;
    if (otherRobotInfo(other_distance, closing_speed, vx_world, vy_world))
    {
      if (other_distance < slow_distance_)
      {
        if (closing_speed > 0.0)
        {
          if (other_distance <= stop_distance_)
          {
            safe_cmd.linear.x = 0.0;
            // Permitir que siga girando para esquivar
            ROS_WARN_THROTTLE(0.5, "[%s] STOP por otro robot a %.2f m",
                              robot_frame_.c_str(), other_distance);
          }
          else
          {
            const double span = std::max(0.01, slow_distance_ - stop_distance_);
            const double scale = min_scale_ +
                (1.0 - min_scale_) * ((other_distance - stop_distance_) / span);
            safe_cmd.linear.x *= clampValue(scale, min_scale_, 1.0);
            ROS_INFO_THROTTLE(1.0, "[%s] slow por otro robot a %.2f m",
                              robot_frame_.c_str(), other_distance);
          }
        }
      }
    }

    cmd_pub_.publish(safe_cmd);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher  cmd_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string global_frame_;
  std::string robot_frame_;
  std::string other_robot_frame_;
  std::string scan_topic_;

  double stop_distance_;
  double slow_distance_;
  double min_scale_;
  double scan_stop_distance_;
  double scan_slow_distance_;
  double scan_front_half_fov_;
  double scan_max_age_;
  bool   enabled_;

  mutable std::mutex scan_mutex_;
  double scan_min_front_;
  double scan_min_left_;
  double scan_min_right_;
  double scan_min_back_;
  double last_scan_time_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_safety_filter");
  CmdVelSafetyFilter filter;
  ros::spin();
  return 0;
}
