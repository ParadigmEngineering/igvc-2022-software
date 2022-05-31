""" gps_listener.py

Listen for GPS messages from the navsat driver and log them.
"""

import rospy
from sensor_msgs.msg import NavSatFix


gps_fix_topic = "fix"


def handle_gps_message(fix: NavSatFix):
    """ Receive and log GPS fix """
    lat = fix.latitude
    long = fix.longitude
    alt = fix.altitude
    rospy.loginfo(f"Lat: {lat} Long: {long} Altitude: {alt}")


def gps_listener_start():
    """ Spin up ros node """
    global gps_fix_topic

    rospy.init_node("gps_listener", anonymous=True, log_level=rospy.INFO)
    rospy.loginfo(f"Listening for GPS fix on topic: {gps_fix_topic}...")
    rospy.Subscriber(gps_fix_topic, NavSatFix, handle_gps_message)
    rospy.spin()


if __name__ == "__main__":
    try:
        gps_listener_start()
    except rospy.ROSInterruptException:
        pass
