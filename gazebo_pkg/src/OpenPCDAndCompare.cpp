#include <iostream>
#include "ros/ros.h"
#include "/opt/ros/hydro/include/pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include "/opt/ros/hydro/include/cv_bridge/cv_bridge.h"
#include "/opt/ros/hydro/include/sensor_msgs/image_encodings.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
cv_bridge::CvImagePtr cvImagePtr;
int i;
std::string filepath;
int row,col;
//cv::Mat_<cv::Point> my_matrix;
cv::Mat_<cv::Point> my_matrix(640, 480, cv::Point(0, 0));
cv::Mat_<cv::Point> img;

void callback(const sensor_msgs::Image &message) {

	std::cout << "Callback" << i << std::endl;

	cvImagePtr = cv_bridge::toCvCopy(message,
			sensor_msgs::image_encodings::TYPE_32FC1);

	std::cout << "Size of ImageMat (Rows, Cols): " << cvImagePtr->image.rows
			<< cvImagePtr->image.cols << std::endl;


//		cv::FileStorage *storage = new cv::FileStorage("mytest.yml",
//				cv::FileStorage::WRITE);
//		*storage << "myimg" << cvImagePtr->image;
//		storage->release();
//		std::cout << "Stored" << std::endl;

}

void PointCloudVectorToMat(){

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filepath, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    std::cout << "Loaded " << std::endl;
    std::cout << "Size of pointcloud vector? : " << cloud->points.size() << std::endl;

    row = 0;
    col = 0;
    for (size_t i = 0; i < cloud->points.size(); i++){
//        std::cout << my_matrix(0) << std::endl;
//    	my_matrix.at(row,col) = cloud->points[i].z;
        my_matrix(i) = cv::Point(0,cloud->points[i].z);
        col++;
        if (col == 639){
            col = 0;
            row++;
        }

    }
    std::cout << "Nr of rows in MAT:" << row << std::endl;

}

void ReadMatFromFile(){
    cv::FileStorage storage("/home/furdek/test.yml", cv::FileStorage::READ);
    storage["myimg"] >> img;
    storage.release();
    std::cout << "Read" << std::endl;
    std::cout << "Rows:" << img.rows << "Columns:" << img.cols << std::endl;

}


int
main (int argc, char** argv)
{

  filepath = "/home/furdek/4gazebo/position0_no_objects.pcd";

  PointCloudVectorToMat();
  ReadMatFromFile();

  return (0);
}

//int main(int argc, char **argv) {
//
//	std::cout << "MAIN IMAGE TO CV" << std::endl;
//	i = 0;
//
//	ros::Subscriber sub;
//	ros::AsyncSpinner *spinner;
//
//	ros::init(argc, argv, "listener");
//	ros::NodeHandle n;
//	sub = n.subscribe("/camera/depth/image_raw", 1000, callback);
//
////	spinner = new ros::AsyncSpinner(1);
////	spinner->start();
//	ros::spin();
//
//	std::cout << "Spinning" << std::endl;
//
//	return 0;
//}
