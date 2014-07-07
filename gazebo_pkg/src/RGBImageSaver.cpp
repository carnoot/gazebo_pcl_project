#include "ros/ros.h"
#include "/opt/ros/hydro/include/pcl_conversions/pcl_conversions.h"
#include "pcl/io/png_io.h"
#include "/opt/ros/hydro/include/openni_camera/openni_image.h"

std::vector<sensor_msgs::Image> pointCloudVector;
pcl::PointCloud<pcl::PointXYZRGB> myPointCloud;
pcl::PCLImage *myPCLImage;
float numberOfCalls;

void callback(sensor_msgs::Image &message) {

    pcl::io::savePNGFile("/home/furdek/forStefanProba.png", myPointCloud);
}

int main(int argc, char **argv) {

    ros::Subscriber sub;
    ros::AsyncSpinner *spinner;

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    sub = n.subscribe("/camera/rgb/image", 1000, callback);

    spinner = new ros::AsyncSpinner(1);
    spinner->start();
    ros::spin();
	return 0;
}
