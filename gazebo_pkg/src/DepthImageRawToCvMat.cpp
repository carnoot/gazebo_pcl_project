#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "/opt/ros/hydro/include/pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include "/opt/ros/hydro/include/cv_bridge/cv_bridge.h"
#include "/opt/ros/hydro/include/sensor_msgs/image_encodings.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
unsigned int k;
int row, col;
std::string filepath;
cv_bridge::CvImagePtr cvImagePtr;
cv::Mat *my_matrix;
float tolerance;
float differences;
//cv::Mat_<cv::Point> my_matrix(640, 480, cv::Point(0, 0));

void PointCloudVectorToMat() {

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) == -1) //* load the file
			{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
	}
	std::cout << "Loaded " << std::endl;
	std::cout << "Size of pointcloud vector? : " << cloud->points.size()
			<< std::endl;

//	my_matrix->create(480, 640, CV_32FC1);

	row = 0;
	col = 0;
    for (int i = 0; i < cloud->points.size(); i++) {

		if (isnan((double) cloud->points[i].z) == 0) {

			my_matrix->at<float>(row, col) = cloud->points[i].z;

//			std::cout << "PointCloud: " << cloud->points[i].z << " Matrix:"
//					<< my_matrix.at<float>(row, col) << std::endl;
		} else {
			my_matrix->at<float>(row, col) = 0;

//			std::cout << "PointCloud NaN: " << cloud->points[i].z << " Matrix:"
//					<< my_matrix.at<float>(row, col) << std::endl;

		}

//        std::cout << my_matrix(0) << std::endl;
//    	my_matrix.at(row,col) = cloud->points[i].z;
//		my_matrix(i) = cv::Point(0, cloud->points[i].z);
//        std::cout << "Value:" << my_matrix.at<float>(row,col) << std::endl;
		col++;
		if (col == 639) {
			col = 0;
			row++;
			std::cout << "Incremented ROW" << std::endl;
		}

	}
	std::cout << "Nr of rows in MAT:" << row << std::endl;

}

void PointCloudComparison() {

	std::cout << "Comparison started" << std::endl;

    for (int i = 0; i < my_matrix->rows; i++) {

        std::cout << i << std::endl;

		for (int j = 0; j < my_matrix->cols; j++) {

            std::cout << "DIFF: " << my_matrix->at<float>(i, j) - cvImagePtr->image.at<float>(i, j) << std::endl;

			if ((my_matrix->at<float>(i, j) - cvImagePtr->image.at<float>(i, j)
					> tolerance)
					|| (my_matrix->at<float>(i, j)
							- cvImagePtr->image.at<float>(i, j) < -tolerance)) {
				differences++;
			}
		}
	}

	std::cout << "Different points: " << differences << std::endl;

}

void callback(const sensor_msgs::Image &message) {

    std::cerr << "CALL" << std::cerr;

    std::cout << "Callback" << k << std::endl;

	std::cout << "Entered conversion function!" << std::endl;

//    my_matrix = new cv::Mat(480,640,CV_32FC1);

	PointCloudVectorToMat();

	std::cout << "Exited conversion function!" << std::endl;

    cvImagePtr = cv_bridge::toCvCopy(message,
            sensor_msgs::image_encodings::TYPE_32FC1);

    std::cout << "Size of ImageMat (Rows, Cols): " << cvImagePtr->image.rows
            << cvImagePtr->image.cols << std::endl;

    PointCloudComparison();

    k++;

}

int main(int argc, char **argv) {

	std::cout << "MAIN IMAGE TO CV" << std::endl;

	filepath = "/home/furdek/4gazebo/position0_no_objects.pcd";
	tolerance = 0.03;
	differences = 0;

    k = 0;
    my_matrix = new cv::Mat(480,640,CV_32FC1);

	ros::Subscriber sub;
	ros::AsyncSpinner *spinner;

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	sub = n.subscribe("/camera/depth/image_raw", 1000, callback);

	std::cout << "Subscribed" << std::endl;

//	spinner = new ros::AsyncSpinner(1);
//	spinner->start();
	ros::spin();

	std::cout << "Spinning" << std::endl;

	return 0;
}

