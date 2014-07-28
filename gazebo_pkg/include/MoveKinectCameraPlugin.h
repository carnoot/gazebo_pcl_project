#include <boost/bind.hpp>
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include <gazebo/common/common.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/Sensor.hh>
#include "gazebo/gazebo.hh"
#include "gazebo_pkg/GetTextParam.h"
#include "ros/ros.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "/home/furdek/catkin_ws/build/gazebo_pkg/custom_pose_message/msgs/custom_pose_request.pb.h"
#include <boost/shared_ptr.hpp>
#include "gazebo_pkg/GetTextParam.h"
#include "gazebo_pkg/ObjectInspectionCenter.h"
#include "ros/ros.h"

namespace gazebo {

class MoveKinectCameraPlugin: public SystemPlugin {

public:
	MoveKinectCameraPlugin();

	virtual ~MoveKinectCameraPlugin();
	virtual void Load(int /*_argc*/, char ** /*_argv*/);
	virtual void Init();

	void OnUpdate();
	void NextPoint();
	void InitCameraNode();
	bool CheckPointReached();
	void CalcAndPublishNextPoint();
	bool getTextParam(
			gazebo_pkg::GetTextParam::Request &req,
			gazebo_pkg::GetTextParam::Response &res);
	void setCameraPosition(
			const boost::shared_ptr<
					const custom_pose_message::msgs::CustomPoseRequest> &);
	bool GetObjectCenter(
			gazebo_pkg::ObjectInspectionCenter::Request &,
			gazebo_pkg::ObjectInspectionCenter::Response &);
	void initService();

private:

	int i, j, k;
	int currentPos;

	double x, y, z;
	double x_offset, y_offset, z_offset;
	double fi, teta;
	double r;
	double stepSize;
	double rotationAngle;
	double rotationStepSize;
	double tolerance;
	double camera_pos_x;
	double camera_pos_y;
	double camera_pos_z;
	double lookAt_x;
	double lookAt_y;
	double lookAt_z;

	Ogre::Entity *myEnt;
	Ogre::Entity *myEnt2;
	Ogre::Entity *myEnt3;
	Ogre::Vector3 posToUse;
	Ogre::Vector3 ogreNewPosVect;
	Ogre::SceneNode *cameraNode;
	Ogre::SceneNode *x_axis, *y_axis, *z_axis;
	Ogre::Animation *cameraAnimation;
	Ogre::Quaternion ogreCameraQuaternion;
	Ogre::SceneManager *sceneManager;
	Ogre::AnimationState *cameraAnimationState;

	rendering::VisualPtr cameraVisual;
	rendering::VisualPtr arrowVisPtr;

	physics::ModelPtr model;

	std::vector<rendering::VisualPtr> visualVect;
	std::vector<math::Pose> posVector;

	std::string cameraLinkName;
	std::string cameraLinkAnimationName;

	math::Pose newPos;
	math::Matrix3 yRotationMatrix;
	math::Matrix3 xRotationMatrix;
	math::Matrix3 pointToRotateMatrix;
	math::Vector3 columnVect;
	math::Vector3 zeroVect;
	math::Matrix3 resultMatrix;
	math::Matrix3 testMatrix1;
	math::Matrix3 testMatrix2;
	math::Quaternion cameraQuaternion;

	event::ConnectionPtr updateConnection;

	transport::NodePtr senderNode;
	transport::NodePtr receiveNode;
	transport::PublisherPtr publisher;
	transport::SubscriberPtr subscriber;

	custom_pose_message::msgs::CustomPoseRequest msgToSend;

	ros::ServiceServer move_camera;
	ros::ServiceServer get_object_center_service;
	ros::AsyncSpinner *spinner;

	bool called;bool present;bool anim;bool published;bool added;bool test_bool;bool look_now;bool position_ready;


};

}

