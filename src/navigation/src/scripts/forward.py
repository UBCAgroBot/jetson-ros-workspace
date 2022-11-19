#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, Float32


class Forward(object):

    def __init__(self):
        self.state = 0

    def state_callback(self, data):
        self.state = data.data

    def publish(self):
        pub_speed = rospy.Publisher('speed', Float32, queue_size=1)
        pub_angle = rospy.Publisher('angle', Float32, queue_size=1)

        rospy.Subscriber("state", Int32, self.state_callback)

        rospy.init_node('forward', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            print(self.state)
            if (self.state == 1):
                pub_speed.publish(1.0)  # 1 is maximum, -1 is maximum reverse, 0 is stop
                pub_angle.publish(0.0)  # negative is left, positive is right. Max is 1.0.

            rate.sleep()


if __name__ == '__main__':
    try:
        forward_node = Forward()
        forward_node.publish()
    except rospy.ROSInterruptException:
        pass
