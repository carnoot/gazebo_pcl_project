#include <gazebo.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "/home/furdek/catkin_ws/build/gazebo_pkg/custom_pose_message/msgs/custom_pose_request.pb.h"
//#include "/home/furdek/catkin_ws/build/gazebo_pkg/custom_pose_message/msgs/custom_pose_request.pb.h"

#include <gazebo_pkg/ObjectInspectionCloud.h>
#include <gazebo_pkg/ObjectInspectionQuaternion.h>
#include <gazebo_pkg/ObjectInspectionCameraPos.h>

#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "ros/ros.h"
#include "/opt/ros/hydro/include/tf/transform_listener.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace gazebo {

//typedef const boost::shared_ptr<geometry_msgs::Pose> customPosePtr;
typedef pcl::PointXYZ PointType;

class MoveCamera: public ModelPlugin {

public:
	MoveCamera();

public:
	virtual ~MoveCamera();

public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
	void NextPoint();

public:
	void OnUpdate();
	void InitCameraNode();
	void FetchRealKinectPose();
	void SetCameraPosition();
	void PrintCameraPose();
	void PrintTransformPose();
	void AdjustPositionToKitchenLink();
	void AdjustRotationToKitchenLink();
	void TransformRotationToEuler();
	void TransformEulerToRotation();
	void SubtractQuaternionAngles();
	void SetTransformQuaternion();
	void SetNewOrientation(const boost::shared_ptr<const custom_pose_message::msgs::CustomPoseRequest> &);
	bool GetCameraPosition(gazebo_pkg::ObjectInspectionCameraPos::Request &,
			gazebo_pkg::ObjectInspectionCameraPos::Response &);
	void SaveClouds(const sensor_msgs::PointCloud2::ConstPtr &);

// Pointer to the model
private:
	physics::ModelPtr model;
	gazebo::msgs::Pose firstPos;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	std::vector<math::Pose> posVector;
	std::string cameraLinkName;
	std::string tfKinectFrameName;
	std::string tfFixedFrameName;
	std::string tfServiceName;
	std::string tfKitchenLinkName;

	std::string urdfPathName;

	int currentPos;
	int i, j;
	float yaw;
	float rotationTest;
	double x, y, z;
	double x_offset, y_offset, z_offset;
	double fi, teta;
	double r;
	double stepSize;
	double rotationAngle;
	double cameraRotationStepSize;
	double kitchen_link_x_offset;
	double kitchen_link_y_offset;
	double kitchen_link_z_offset;
	double kitchen_link_roll_offset;
	double kitchen_link_pitch_offset;
	double kitchen_link_yaw_offset;
	double rollAngle;
	double pitchAngle;
	double yawAngle;
	double rotationW;
	double rotationX;
	double rotationY;
	double rotationZ;
	double look_at_pos_x;
	double look_at_pos_y;
	double look_at_pos_z;
	double look_at_w;
	double look_at_x;
	double look_at_y;
	double look_at_z;
	double delay;

	bool orientation_ready;
	bool get_cloud_ready;
	bool last_pose;

	math::Pose newPos;
	math::Pose probaPos;
	math::Quaternion rotationQuaternion;
	math::Quaternion initialQuaternion;
	math::Quaternion auxQuaternion;
	math::Quaternion finalQuaternion;
	math::Quaternion testQuaternion_1;
	math::Quaternion testQuaternion_2;
	math::Matrix3 yRotationMatrix;
	math::Matrix3 xRotationMatrix;
	math::Matrix3 pointToRotateMatrix;
	math::Vector3 columnVect;
	math::Vector3 zeroVect;
	math::Matrix3 resultMatrix;
	math::Matrix3 testMatrix1;
	math::Matrix3 testMatrix2;

    pcl::PointCloud<PointType> cloud;

public:
	ros::ServiceServer service;
	ros::ServiceServer service_1;
	ros::ServiceClient get_cloud_client;
	ros::ServiceClient pass_camera_position_client;
	ros::ServiceClient cam_quaternion_client;
	ros::ServiceClient object_number_client;
	ros::Subscriber sub;
	event::ConnectionPtr updateConnection;
	transport::NodePtr receiveNode;
	transport::NodePtr senderNode;
	transport::SubscriberPtr subscriber;
	transport::PublisherPtr publisher;
	custom_pose_message::msgs::CustomPoseRequest msgToSend;

};

}
