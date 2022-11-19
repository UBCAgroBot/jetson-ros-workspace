#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32


def publish():
    pub = rospy.Publisher('state', Int32, queue_size=1)
    rospy.init_node('decision', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub.publish(1)  # 0 - stop, 1 - forward, 2 - turn
        rate.sleep()


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
