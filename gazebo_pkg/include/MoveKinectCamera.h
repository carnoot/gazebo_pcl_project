#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <gazebo.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <stdio.h>
#include "ros/ros.h"
#include "/opt/ros/hydro/include/tf/transform_listener.h"

namespace gazebo {

//typedef const boost::shared_ptr<geometry_msgs::Pose> customPosePtr;

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
//	void SetNewOrientation(customPosePtr);

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
	math::Pose newPos;
	math::Pose probaPos;
	math::Quaternion rotationQuaternion;
	math::Quaternion initialQuaternion;
	math::Quaternion auxQuaternion;
	math::Quaternion finalQuaternion;
	math::Matrix3 yRotationMatrix;
	math::Matrix3 xRotationMatrix;
	math::Matrix3 pointToRotateMatrix;
	math::Vector3 columnVect;
	math::Vector3 zeroVect;
	math::Matrix3 resultMatrix;
	math::Matrix3 testMatrix1;
	math::Matrix3 testMatrix2;

public:
	event::ConnectionPtr updateConnection;
	transport::NodePtr receiveNode;
	transport::SubscriberPtr subscriber;

};

}
