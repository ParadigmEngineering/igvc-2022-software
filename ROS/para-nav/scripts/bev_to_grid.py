""" bev_to_grid.py

Generate an occupancy grid from a segmented birds eye view.

Subscribes:
    /bev/segmented
Publishes: 
    /bev/occupancy-grid 
"""

import rospy
from cv_bridge import CvBridge, CvBridgeError


def main():
    """ Initialize and execute ROS node """
    raise NotImplementedError

if __name__ == "__main__":
    main()
