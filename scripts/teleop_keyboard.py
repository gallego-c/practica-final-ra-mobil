#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

MOVE_BINDINGS = {
    "w": (1.0, 0.0),
    "x": (-1.0, 0.0),
    "a": (0.0, 1.0),
    "d": (0.0, -1.0),
    "q": (0.0, 1.0),
    "e": (0.0, -1.0),
    "s": (0.0, 0.0),
}

HELP_TEXT = """
Keyboard Teleop (TurtleBot Waffle)
-------------------------------
W/X : forward/back
A/D : turn left/right
S   : stop
Q/E : turn left/right (same as A/D)
CTRL-C to quit
"""


def get_key(timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)
    return key


def stop_robot(pub):
    twist = Twist()
    pub.publish(twist)


def main():
    rospy.init_node("teleop_keyboard")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    speed = rospy.get_param("~speed", 0.4)
    turn = rospy.get_param("~turn", 1.0)

    rospy.on_shutdown(lambda: stop_robot(pub))

    print(HELP_TEXT)
    rate = rospy.Rate(10)
    x = 0.0
    z = 0.0

    while not rospy.is_shutdown():
        key = get_key(0.1)

        if key in MOVE_BINDINGS:
            x, z = MOVE_BINDINGS[key]
        elif key == "\x03":
            break
        else:
            x, z = 0.0, 0.0

        twist = Twist()
        twist.linear.x = x * speed
        twist.angular.z = z * turn
        pub.publish(twist)
        rate.sleep()

    stop_robot(pub)


if __name__ == "__main__":
    SETTINGS = termios.tcgetattr(sys.stdin)
    try:
        main()
    except Exception:
        stop_robot(rospy.Publisher("cmd_vel", Twist, queue_size=1))
        raise
