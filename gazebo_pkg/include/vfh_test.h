#ifndef PCL_PROCESSING_MODULE_H
#define PCL_PROCESSING_MODULE_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "gazebo_pkg/ObjectInspectionCloud.h"
#include "gazebo_pkg/ObjectInspectionBounding.h"
#include "gazebo_pkg/ObjectInspectionQuaternion.h"
#include "gazebo_pkg/ObjectCanSendNextCamPos.h"

#include "ml_classifiers/CreateClassifier.h"
#include "ml_classifiers/AddClassData.h"
#include "ml_classifiers/TrainClassifier.h"
#include "ml_classifiers/ClassifyData.h"
#include "ml_classifiers/ClassDataPoint.h"

#include "pcl/pcl_base.h"
#include "pcl/io/pcd_io.h"
#include "pcl/common/common.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include "pcl_conversions/pcl_conversions.h"
#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>

#include <gazebo/math/Quaternion.hh>
#include <gazebo.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include <pcl/common/transforms.h>

#include <boost/filesystem.hpp>
#include <flann/flann.h>
//#include <flann/io/hdf5.h>
#include <fstream>
#include <mutex>
#include <eigen3/Eigen/Core>

#include <iostream>

typedef pcl::PointXYZRGB PointType;
typedef std::pair<std::string, std::vector<float> > vfh_model;

class VFHTest {

public:
	VFHTest();
	virtual ~VFHTest();

	bool GetCloud(gazebo_pkg::ObjectInspectionCloud::Request &req,
			gazebo_pkg::ObjectInspectionCloud::Response &res);
	void SaveClouds(const sensor_msgs::PointCloud2::ConstPtr &);

	void DisplayPoints(pcl::PointCloud<PointType> &);
	void DisplayResults();
	void ProcessResults();

	int TraversePCDDirectory(const boost::filesystem::path &);

	bool HasDirectories(const boost::filesystem::path &);

	int TraverseVFHDirectory(const boost::filesystem::path &);

	void SaveVFHS(std::string file_path,
			pcl::PointCloud<pcl::VFHSignature308>::Ptr);

	pcl::PointCloud<pcl::VFHSignature308>::Ptr ComputeVFH(
			pcl::PointCloud<PointType>);

	void CreateVFHFiles(const boost::filesystem::path &, const std::string &,
			std::string);

	void CreateFileForLibSVM(std::string &, std::string &);

	void GenerateLibSVMCommand(std::string &, std::string &, std::string &);
	void GenerateLibSVMCommand(std::string &, std::string &, std::string &,
			std::string &);

	bool loadHist(const boost::filesystem::path &, vfh_model &);

	int CreateSVMClassifier();

	int AddSVMClassData();

	int TrainSVMData();

	std::vector<std::string> GetNumberOfSVMDataTypes();

	float CalculatePercentageOfFitness(float, std::string,
			std::vector<std::string>);

	void SeparateSVMData(std::vector<std::string> class_types_vector);

	int ClassifySVMData();

	int ClassifySeparateSVMData();

	void CreateSCVFile(std::string &);

	void LoadSCVFile(std::string &);

	pcl::PointCloud<pcl::Normal>::Ptr normalEstimation(
			const pcl::PointCloud<PointType>::ConstPtr);

public:

	std::vector<int> result_vect;
	std::vector<vfh_model> models;
	std::vector<int> model_label_values;

	std::vector<std::vector<double>> data_points;
	std::vector<std::string> data_points_labels;
	std::vector<std::string> result_labels;
	std::vector<ml_classifiers::ClassDataPoint> data;
	std::vector<std::vector<ml_classifiers::ClassDataPoint>> separated_data;
	ml_classifiers::ClassDataPoint classPoint;

	int vfh_files_found;
	int test_counter;
	int label;
	float bounding_min[3];
	float bounding_max[3];
	Eigen::Vector4f min_pt;
	Eigen::Vector4f max_pt;

	std::mutex mut;
	std::string primary_folder_path;
	std::string extension_to_read;
	std::string csv_training_file_name;
	std::string csv_testing_file_name;
	std::string csv_training_file_1_name;
	std::string libSVM_training_file_name;
	std::string libSVM_testing_file_name;
	std::string svm_identifier;
	std::string svm_type_classifier;
	std::string libSVM_model_file_name;
	std::string libSVM_results_file_name;
	std::string libSVM_svm_predict_exe_name;
	std::string libSVM_svm_train_exe_name;

	ros::ServiceServer service;
	ros::ServiceServer service_1;
	ros::ServiceServer get_cam_quaternion;
	ros::ServiceClient can_send_next_pos;
	ros::ServiceClient create_svm_classifier;
	ros::ServiceClient add_svm_class_data;
	ros::ServiceClient train_svm_classifier;
	ros::ServiceClient classify_svm_data;

	ros::Subscriber sub;

	pcl::PointCloud<PointType> cloud;
	pcl::PointCloud<PointType> cloud_to_process;
	pcl::PointCloud<PointType> *cloud_ptr;
	pcl::PointCloud<PointType> cloud_after_processing;
	pcl::PointCloud<PointType> rotated_cloud_to_process;
	pcl::PointCloud<PointType> rotated_axis_cloud_to_process;
	pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;

	gazebo::math::Quaternion *cam_quaternion;
	gazebo::math::Quaternion *aux_quaternion;
	Eigen::Matrix4f transform_matrix;
	Eigen::Matrix4f transform_matrix_axis;

	double quaternion_values[4];

	bool can_process;bool x_axis_ok;bool y_axis_ok;bool z_axis_ok;

};

#endif
