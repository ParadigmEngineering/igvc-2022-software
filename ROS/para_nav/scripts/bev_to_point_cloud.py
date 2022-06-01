""" bev_point_cloud 

Generate a point cloud from a BEV
"""

import rospy 
from sensor_msgs.msg import Image, PointCloud2


pc_pub = rospy.Publisher("point_cloud/segmented")


def image_to_point_cloud(image: Image) -> PointCloud2:
    """ Convert an Image message to a PointCloud2 message """
    pass


def handle_bev(image: Image):
    """ Handle incoming BEV image """
    pass


def main():
    """ Spin up node """
    rospy.init_node("bev_to_point_cloud")
    rospy.Subscriber("bev/segmented", Image, handle_bev)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted, closing...")
