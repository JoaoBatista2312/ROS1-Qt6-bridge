#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def increment_publisher():
    pub = rospy.Publisher('/chatter', String, queue_size=10)
    rospy.init_node('increment_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 50 message per second
    count = 0

    while not rospy.is_shutdown():
        message = f"Count: {count}"
        rospy.loginfo(message)
        pub.publish(String(data=message))
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        increment_publisher()
    except rospy.ROSInterruptException:
        pass
