#!/usr/bin/env python
""" bev_publisher.py

Publish birds eye view images on an interval 
for testing purposes. 
"""

import rospy
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def load_image(filepath: str) -> cv2.Mat:
    """ Load image at filepath using opencv """
    img = cv2.imread(filepath, cv2.IMREAD_ANYCOLOR)
    return img


def cv_to_ros_image(cv_img: cv2.Mat) -> Image:
    """ Convert an opencv image to a ROS image """
    ros_image = Image()

    try:
        cv_bridge = CvBridge()
        ros_image = cv_bridge.cv2_to_imgmsg(cv_img, encoding = "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
    
    return ros_image


def main():
    """ Initialize and execute ROS node """
    rospy.init_node("bev_publisher", anonymous=True)
    filepath = rospy.get_param("/para/bev_publisher_node/filepath")    
    topic = "bev/segmented"
    pub = rospy.Publisher(topic, Image, queue_size=10)
    
    # Load image with opencv and convert to ros message
    cv_img = load_image(filepath)
    ros_img = cv_to_ros_image(cv_img)
    ros_img.header.frame_id = "zed_2_camera_center"
    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():    
        # Publish image
        pub.publish(ros_img) 
        rospy.loginfo(f"Published segmented BEV image\n Topic = {topic}, File = {filepath}")
        rate.sleep()


if __name__ == "__main__":
    main()
