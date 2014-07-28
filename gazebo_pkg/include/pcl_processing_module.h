#ifndef PCL_PROCESSING_MODULE_H
#define PCL_PROCESSING_MODULE_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "gazebo_pkg/ObjectInspectionCloud.h"
#include "gazebo_pkg/ObjectInspectionBounding.h"
#include "gazebo_pkg/ObjectInspectionQuaternion.h"
#include "gazebo_pkg/ObjectInspectionClassifier.h"
#include "gazebo_pkg/ObjectInspectionClassifyClouds.h"
#include "gazebo_pkg/ObjectCanSendNextCamPos.h"
#include "gazebo_pkg/VFHTestCorrectIndexes.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/pcl_base.h"
#include "pcl/io/pcd_io.h"
#include "pcl/common/common.h"
#include <mutex>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <gazebo/math/Quaternion.hh>
#include <gazebo.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include <pcl/common/transforms.h>
//#include <eigen3/Eigen/src/Core/MatrixBase.h>
//#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

typedef pcl::PointXYZRGB PointType;

class PclProcesser {

public:
	PclProcesser(int, char **);
	virtual ~PclProcesser();

	bool GetCloud(gazebo_pkg::ObjectInspectionCloud::Request &req,
			gazebo_pkg::ObjectInspectionCloud::Response &res);
	bool GetCamQuaternion(
			gazebo_pkg::ObjectInspectionQuaternion::Request &,
			gazebo_pkg::ObjectInspectionQuaternion::Response &);
	void SaveClouds(const sensor_msgs::PointCloud2::ConstPtr &);
	void SendCloudsToBeClassifed();
	void EraseBadClassifiers();
	bool GetObjectBounding(
			gazebo_pkg::ObjectInspectionBounding::Request &,
			gazebo_pkg::ObjectInspectionBounding::Response &);
	bool GetInCorrectIndexes(gazebo_pkg::VFHTestCorrectIndexes::Request &,
			gazebo_pkg::VFHTestCorrectIndexes::Response &);
	int PointsInBoundingBoxManual(pcl::PointCloud<PointType>);
	int PointsInBoundingBoxPcl(pcl::PointCloud<PointType>);
	void FindMaxElements(int);
	void CreateFinalCloudVector();
	void PlaneSegmentationExtraction(const pcl::PointCloud<PointType>::Ptr);
	void DisplayPoints(pcl::PointCloud<PointType> &);
	void DisplayResults(std::vector<int>);
	void SaveCloudPCDs();

	int clouds_processed;
	int first_max_elements;
	int final_cloud_to_save_contor;

	float bounding_min[3];
	float bounding_max[3];

	double quaternion_values[4];

	std::vector<int> result_vect;
	std::vector<int> best_positions;
	std::vector<int> best_positions_indexes;
	std::vector<int> incorrect_indexes;
	std::vector<pcl::PointCloud<PointType>> clouds_to_process_vect;

	std::mutex mut;
	std::string my_classifier;
	std::string orig_cloud_to_save_base_path;
	std::string final_cloud_to_save_base_path;

	Eigen::Vector4f min_pt;
	Eigen::Vector4f max_pt;
	Eigen::Matrix4f transform_matrix;
	Eigen::Matrix4f transform_matrix_axis;

	gazebo::math::Quaternion *cam_quaternion;
	gazebo::math::Quaternion *aux_quaternion;

	ros::Subscriber camera_depth_points_sub;

	ros::ServiceServer get_cloud;
	ros::ServiceServer get_object_bounding;
	ros::ServiceServer get_cam_quaternion;
	ros::ServiceServer get_classifier;
	ros::ServiceServer get_correct_indexes;

	ros::ServiceClient can_send_next_pos;
	ros::ServiceClient send_clouds_to_classify;

	pcl::PointCloud<PointType> cloud;
	pcl::PointCloud<PointType> cloud_to_process;
	pcl::PointCloud<PointType> *cloud_ptr;
	pcl::PointCloud<PointType> cloud_after_processing;
	pcl::PointCloud<PointType> rotated_cloud_to_process;
	pcl::PointCloud<PointType> rotated_axis_cloud_to_process;
	pcl::PointCloud<PointType> filtered_cloud;

	bool can_process;bool get_best_positions;bool x_axis_ok;bool y_axis_ok;bool z_axis_ok;bool erase_bad_classifiers;

};

#endif
