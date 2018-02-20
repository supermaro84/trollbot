#!/usr/bin/env python

import rospy
from math import sin, cos

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class BasicController:
    def __init__(self):
        self._linear = 0
        self._angular = 0

        rospy.Subscriber("cmd_vel", Twist, self.twistCallback)
        self._publ = rospy.Publisher("left_wheel_speed", Float32, queue_size=10)
        self._pubr = rospy.Publisher("right_wheel_speed", Float32, queue_size=10)

    def twistCallback(self, data):
        self._linear = data.linear.x
        self._angular = data.angular.z

    def publish_v(self):
        v_left = self._linear*255 - self._angular*255
        v_right = self._linear*255 + self._angular*255

        self._publ.publish(v_left)
        self._pubr.publish(v_right)

def main():
    rospy.init_node("basic_controller")

    basic_controller = BasicController()

    rate = rospy.Rate(20)

    rospy.loginfo("Running basic controller")
    while not rospy.is_shutdown():
        basic_controller.publish_v()
        rate.sleep()


if __name__ == '__main__':
    main()
