import rospy
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from mip_rover_lowlevel.msg import MotorCmd, MotorRate


class Driver:
    def __init__(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not rospy.has_param('/mip_rover/gemoetry/l'):
            rate.sleep()
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException('never started')

        self._l = rospy.get_param('/mip_rover/geometry/l')
        self._r = rospy.get_param('/mip_rover/geometry/r')

        self._motor_gain = np.array(rospy.get_param('/mip_rover/motor_controller/motor_gain'))
        self._motor_bias = np.array(rospy.get_param('/mip_rover/motor_controller/motor_bias'))

        self._current_motor_rate = MotorRate()
        self._current_motor_cmd = MotorCmd()

        self._twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_handler)
        self._motor_rate_sub = rospy.Subscriber('/mip_rover/motors/rates', MotorRate, self.motor_rate_handler)

        self._motor_cmd_pub = rospy.Publisher('/mip_rover/motors/cmd', MotorCmd, queue_size=10)

        self.motors_off()

    def motor_rate_handler(self, motor_rate_msg):
        self._current_motor_rate = motor_rate_msg

    def twist_handler(self, twist_msg):

        vel_des = np.zeros([2, 1])
        vel_des[0, 0] = (twist_msg.linear.x - (twist_msg.angular.z * (self._l / 2))) / self._r
        vel_des[1, 0] = (twist_msg.linear.x + (twist_msg.angular.z * (self._l / 2))) / self._r
        vel_des = self._motor_gain * vel_des + self._motor_bias

        self.send_motor_cmd(vel_des[0, 0], vel_des[1, 0])

        self._last_cmd_time = rospy.Time.now()

    def motors_off(self):
        self.send_motor_cmd(0, 0)

    def send_motor_cmd(self, left, right):
        self.last_cmd_time = rospy.Time.now();
        self._current_motor_cmd.header.stamp = self.last_cmd_time
        self._current_motor_cmd.left_motor = left
        self._current_motor_cmd.right_motor = right
        self._motor_cmd_pub.publish(self._current_motor_cmd)
