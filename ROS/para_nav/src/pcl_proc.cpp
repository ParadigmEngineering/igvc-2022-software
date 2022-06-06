#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include <unordered_map>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
ros::Publisher pub;

// <SLabel Name="road"             fromColour=" 81  0   81"    toValue=" 1" />
// <SLabel Name="lane line"        fromColour="250 170  30"    toValue=" 2" />
// <SLabel Name="obstacle"         fromColour="  0   0   0"    toValue=" 3" />
// <SLabel Name="person"           fromColour="255   0   0"    toValue=" 4" />
// <SLabel Name="car"              fromColour="255 255 255"    toValue=" 5" />
// <SLabel Name="building"         fromColour=" 70  70  70"    toValue=" 6" />
// <SLabel Name="terrain"          fromColour="152 251 152"    toValue=" 6" />
// <SLabel Name="pot hole"         fromColour="150 120  90"    toValue=" 7" />
// <SLabel Name="sky"              fromColour=" 70 130 180"    toValue=" 8" />
// <SLabel Name="vegetation"       fromColour="107 142  35"    toValue=" 9" />
// <SLabel Name="ramp"             fromColour="230 150 140"    toValue=" 10" />

void handle_pcl(const PointCloud::ConstPtr& pcl)
{
 	PointCloud::Ptr pcl_out(new PointCloud);

	const double PPM = 88.29;
	size_t z = 0;
    
	for (const auto& point: pcl->points)
	{
		pcl::PointXYZRGB pt = point;
		pt.y = 0;
		pcl_out->push_back(pt);
	}

	pcl_out->header.frame_id = pcl->header.frame_id;
	pcl_out->width = pcl->width;
	pcl_out->height = pcl->height;

    pub.publish(pcl_out);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_cleanup");
	
	ros::NodeHandle nh;
 	pub = nh.advertise<PointCloud>("pcl/seg/cost_pcl", 1);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<PointCloud>("point_cloud/points", 1, handle_pcl);
	ros::spin();
}
