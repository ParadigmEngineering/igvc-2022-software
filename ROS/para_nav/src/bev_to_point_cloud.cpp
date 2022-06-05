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

void image_to_pcl(const sensor_msgs::ImageConstPtr& image, const PointCloud::Ptr& pcl)
{
	const double PPM = 88.29;
	const double OBSTACLE_HEIGHT = 0;
	const double REGULAR_HEIGHT = 0;

	cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvShare(image, "bgr8");

	size_t z = 0;
	for (size_t y = 0; y < cv_img->image.rows; y++)
	{
		for (size_t x = 0; x < cv_img->image.cols; x++)
		{
			auto color = cv_img->image.at<cv::Vec3b>(cv::Point(x, y));
		
			if ( color == cv::Vec3b(81, 0, 81) || color == cv::Vec3b(150, 150, 150))
			{
				z = -100;
			}
			else
			{
				z = 0;
			}

			auto p = pcl::PointXYZRGB();
			p.x = (x / PPM) - 2.8995; // Center image in coordinate frame 
			p.y = (y / PPM) + 0.694; 
			p.z = z;
			p.r = color[2];
			p.g = color[1];
			p.b = color[0];

			pcl->points.push_back(p);
		}
	}
}

void handle_image(const sensor_msgs::ImageConstPtr& image)
{
	PointCloud::Ptr msg (new PointCloud);
	
	msg->header.frame_id = "base_link";
	msg->height = image->height; 
	msg->width = image->width;
	image_to_pcl(image, msg);
	pub.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bev_to_point_cloud");
	cv::namedWindow("view");
	
	ros::NodeHandle nh;
 	pub = nh.advertise<PointCloud>("pcl/segmented", 1);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("bev/segmented", 1, handle_image);
	ros::spin();
	cv::destroyWindow("view");
}
