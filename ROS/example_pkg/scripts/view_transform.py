#!/usr/bin/env python
""" talker.py
Simple talker demo that publishes std_msgs/Strings messages
to the 'chatter' topic.
"""


import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def handle_imu_data(imu: Imu):
    quaternion = imu.orientation
    explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = euler_from_quaternion(explicit_quat)
    print(math.degrees(euler[1]))

def talker():
    # 'talker':   name of ros node, must not contain slashes
    # anon: Ensures unique name by adding random numbers to the end, 
    rospy.init_node('pitch_viewer', anonymous=True, log_level=rospy.INFO)
    pub = rospy.Subscriber("/zed2/zed_node/imu/data", Imu, handle_imu_data)
    # Rate object, convenience for looping at desired rate via rate.sleep()
    rate = rospy.Rate(10) # 10hz

    # Standard rospy op loop 
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass