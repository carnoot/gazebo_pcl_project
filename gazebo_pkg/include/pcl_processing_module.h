
#ifndef PCL_PROCESSING_MODULE_H
#define PCL_PROCESSING_MODULE_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "gazebo_pkg/ObjectInspectionCloud.h"
#include "gazebo_pkg/ObjectInspectionBounding.h"
#include "gazebo_pkg/ObjectInspectionQuaternion.h"
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
#include <pcl/filters/extract_indices.h>
#include <gazebo/math/Quaternion.hh>
//#include <eigen3/Eigen/src/Core/MatrixBase.h>
//#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

typedef pcl::PointXYZ PointType;

class PclProcesser{

public:
	PclProcesser(int, char **);
    virtual ~PclProcesser();

    bool GetCloud(gazebo_pkg::ObjectInspectionCloud::Request &req,
    		gazebo_pkg::ObjectInspectionCloud::Response &res);
    bool GetCamQuaternion(
    		gazebo_pkg::ObjectInspectionQuaternion::Request &,
    		gazebo_pkg::ObjectInspectionQuaternion::Response &);
    void SaveClouds(const sensor_msgs::PointCloud2::ConstPtr &);
    bool GetObjectBounding(
    		gazebo_pkg::ObjectInspectionBounding::Request &,
    		gazebo_pkg::ObjectInspectionBounding::Response &);
    int PointsInBoundingBoxManual(pcl::PointCloud<PointType>);
    int PointsInBoundingBoxPcl(pcl::PointCloud<PointType>);
    void PlaneSegmentationExtraction(const pcl::PointCloud<PointType>::Ptr);
    void DisplayPoints();

public:

    float bounding_min[3];
    float bounding_max[3];
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;

    std::mutex mut;

    ros::ServiceServer service;
    ros::ServiceServer service_1;
    ros::ServiceServer get_cam_quaternion;
    ros::Subscriber sub;
    pcl::PointCloud<PointType> cloud;
    pcl::PointCloud<PointType> cloud_to_process;
    pcl::PointCloud<PointType> *cloud_ptr;
    pcl::PointCloud<PointType> cloud_after_processing;

    gazebo::math::Quaternion *cam_quaternion;
    gazebo::math::Quaternion *aux_quaternion;

	double quaternion_values[4];

    bool can_process;
    bool x_axis_ok;
    bool y_axis_ok;
    bool z_axis_ok;

};

#endif
