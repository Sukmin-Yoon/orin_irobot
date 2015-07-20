#!/usr/bin/python
import roslib; roslib.load_manifest('irobot_teleop')
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class IrobotTeleop:
    def __init__(self):
        self.cmdPub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        self.joySub = rospy.Subscriber('joy', Joy, self.joyCbk, queue_size=10)
        self.scale_linear = rospy.get_param('~scale_linear', 1)
        self.scale_angular = rospy.get_param('~scale_angular', 1)
        self.axis_linear = rospy.get_param('~axis_linear', 3)
        self.axis_angular = rospy.get_param('~axis_angular', 0)

    def joyCbk(self, msg):
        cmd = Twist()
        cmd.angular.z = msg.axes[self.axis_angular] * self.scale_angular
        cmd.linear.x = msg.axes[self.axis_linear] * self.scale_linear
        self.cmdPub.publish(cmd)


if __name__ == "__main__":
    node = rospy.init_node('irobot_teleop')

    t = IrobotTeleop()

    rospy.spin()