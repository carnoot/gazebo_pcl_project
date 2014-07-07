#include "ros/ros.h"
#include "/opt/ros/hydro/include/pcl_conversions/pcl_conversions.h"
#include "pcl/io/png_io.h"
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include "/opt/ros/hydro/include/cv_bridge/cv_bridge.h"

std::vector<sensor_msgs::PointCloud2> pointCloudVector;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr myPointCloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
float numberOfCalls;
pcl::PointXYZRGB *myPoint;
pcl::PointXYZRGB *myPoint2;
std::string filename;
//pcl::PointXYZRGB *myPoint;
cv_bridge::CvImagePtr cvImagePtr;


void callback(const sensor_msgs::PointCloud2::ConstPtr &message) {

//	pointCloudVector.push_back(*message);
//	cvImagePtr = cv_bridge::toCvCopy()
    pcl::fromROSMsg(*message, *cloud);
}

int main(int argc, char **argv) {

	ros::Subscriber sub;
	ros::AsyncSpinner *spinner;

    filename = "/home/furdek/PCDs/table_scene_lms400.pcd";

//    pcl::PCDReader reader;
//    reader.read("/home/furdek/PCDs/table_scene_lms400.pcd", *cloud);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud) == -1) //* load the file
            {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
                new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
                cloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, filename);
        viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, filename);
        viewer->initCameraParameters();
        viewer->saveScreenshot("/home/furdek/StefanScreenshot.png");
        while(1){

        }

//	ros::init(argc, argv, "listener");
//	ros::NodeHandle n;
//	sub = n.subscribe("/camera/depth/points", 1000, callback);

//	spinner = new ros::AsyncSpinner(1);
//	spinner->start();
//	ros::spin();
	return 0;
}

