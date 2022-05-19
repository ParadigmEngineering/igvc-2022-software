#!/usr/bin/env python
""" talker.py
Simple talker demo that publishes std_msgs/Strings messages
to the 'chatter' topic.
"""

import rospy
from std_msgs.msg import String


def talker():
    # Queue size -> limits the amount of queued messages
    # If any subscriber is not receiving them fast enough
    pub = rospy.Publisher('chatter', String, queue_size=10)

    # 'talker':   name of ros node, must not contain slashes
    # anon: Ensures unique name by adding random numbers to the end, 
    rospy.init_node('test_talker', anonymous=True, log_level=rospy.INFO)

    # Rate object, convenience for looping at desired rate via rate.sleep()
    rate = rospy.Rate(10) # 10hz

    # Standard rospy op loop 
    while not rospy.is_shutdown():
        # Note: message constructors take arguments in the 
        # order defined in the .msg file 
        hello_str = f"Hello world! {rospy.get_time()}"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
