#include "ros/ros.h"
#include "/opt/ros/hydro/include/pcl_conversions/pcl_conversions.h"
#include "pcl/io/png_io.h"
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include "/opt/ros/hydro/include/cv_bridge/cv_bridge.h"

typedef pcl::PointXYZ PointT;

std::vector<sensor_msgs::PointCloud2> pointCloudVector;
pcl::PointCloud<PointT>::Ptr myPointCloud;
pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
float numberOfCalls;
pcl::PointXYZRGB *myPoint;
pcl::PointXYZRGB *myPoint2;
std::string filename;
//pcl::PointXYZRGB *myPoint;
cv_bridge::CvImagePtr cvImagePtr;

std::string rosTopicName;


void callback(const sensor_msgs::PointCloud2::ConstPtr &message) {

//	pointCloudVector.push_back(*message);
//	cvImagePtr = cv_bridge::toCvCopy()
    pcl::fromROSMsg(*message, *cloud);
    pcl::PCDWriter writer;
    writer.write("/home/furdek/herle_original.pcd",*cloud);
    std::cerr << "Written Herle Cloud" << std::endl;
}

int main(int argc, char **argv) {

	ros::Subscriber sub;
	ros::AsyncSpinner *spinner;

    filename = "/home/furdek/PCDs/table_scene_lms400.pcd";
    rosTopicName = "/kinect_head/depth_registered/points";

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    sub = n.subscribe(rosTopicName, 1000, callback);

//	spinner = new ros::AsyncSpinner(1);
//	spinner->start();
    ros::spin();
	return 0;
}

