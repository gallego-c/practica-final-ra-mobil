#include <signal.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace {

struct KeyConfig {
  double linear;
  double angular;
};

KeyConfig getBinding(char key, bool* known) {
  *known = true;
  switch (key) {
    case 'w':
      return {1.0, 0.0};
    case 'x':
      return {-1.0, 0.0};
    case 'a':
      return {0.0, 1.0};
    case 'd':
      return {0.0, -1.0};
    case 'q':
      return {0.0, 1.0};
    case 'e':
      return {0.0, -1.0};
    case 's':
      return {0.0, 0.0};
    default:
      *known = false;
      return {0.0, 0.0};
  }
}

class KeyboardReader {
 public:
  KeyboardReader() {
    tcgetattr(STDIN_FILENO, &stored_);
    raw_ = stored_;
    raw_.c_lflag &= ~(ICANON | ECHO);
    raw_.c_cc[VMIN] = 0;
    raw_.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_);
  }

  ~KeyboardReader() { tcsetattr(STDIN_FILENO, TCSANOW, &stored_); }

  bool readKey(char* key, double timeout_s) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    timeval timeout;
    timeout.tv_sec = static_cast<int>(timeout_s);
    timeout.tv_usec = static_cast<int>((timeout_s - timeout.tv_sec) * 1000000);

    int rv = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);
    if (rv > 0 && FD_ISSET(STDIN_FILENO, &set)) {
      char c;
      if (read(STDIN_FILENO, &c, 1) > 0) {
        *key = c;
        return true;
      }
    }
    return false;
  }

 private:
  termios stored_;
  termios raw_;
};

const char kHelp[] =
    "Keyboard Teleop (TurtleBot Waffle)\n"
    "-------------------------------\n"
    "W/X : forward/back\n"
    "A/D : turn left/right\n"
    "S   : stop\n"
    "Q/E : turn left/right (same as A/D)\n"
    "CTRL-C to quit\n";

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_keyboard");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double speed = 0.4;
  double turn = 1.0;
  pnh.getParam("speed", speed);
  pnh.getParam("turn", turn);

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  std::puts(kHelp);

  KeyboardReader reader;
  ros::Rate rate(10);

  while (ros::ok()) {
    char key = 0;
    bool got = reader.readKey(&key, 0.1);

    double x = 0.0;
    double z = 0.0;
    if (got) {
      if (key == 3) {
        break;
      }
      bool known = false;
      KeyConfig cfg = getBinding(key, &known);
      if (known) {
        x = cfg.linear;
        z = cfg.angular;
      }
    }

    geometry_msgs::Twist twist;
    twist.linear.x = x * speed;
    twist.angular.z = z * turn;
    pub.publish(twist);

    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::Twist stop;
  pub.publish(stop);
  return 0;
}
