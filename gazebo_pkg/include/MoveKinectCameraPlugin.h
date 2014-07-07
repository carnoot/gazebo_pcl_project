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
#include "ros/ros.h"

namespace gazebo {

/// \SystemGui Class
class MoveKinectCameraPlugin: public SystemPlugin {

	/// \ Constructor
public:
	MoveKinectCameraPlugin();

public:
	virtual ~MoveKinectCameraPlugin();

/// \Load

public:
	virtual void Load(int /*_argc*/, char ** /*_argv*/);
	void NextPoint();

private:
	virtual void Init();

public:
	void OnUpdate();
	void InitCameraNode();
	bool CheckPointReached();
	void CalcAndPublishNextPoint();
	bool getTextParam(gazebo_pkg::GetTextParam::Request &req,
			gazebo_pkg::GetTextParam::Response &res);
	void initService();

	// Pointer to the model
private:
	physics::ModelPtr model;
	rendering::VisualPtr cameraVisual;
	rendering::VisualPtr arrowVisPtr;
	Ogre::Entity *myEnt;
	Ogre::Entity *myEnt2;
	Ogre::Entity *myEnt3;
	std::vector<rendering::VisualPtr> visualVect;
	Ogre::SceneManager *sceneManager;
	gazebo::sensors::SensorPtr cameraSensor;
	Ogre::SceneNode *cameraNode;
	math::Quaternion cameraQuaternion;
	Ogre::Quaternion ogreCameraQuaternion;
	Ogre::Vector3 ogreNewPosVect;
	Ogre::SceneNode *x_axis, *y_axis, *z_axis;
	std::vector<math::Pose> posVector;
	int currentPos;
	std::string cameraLinkName;
	std::string cameraLinkAnimationName;
	//Ogre::SceneManager::AnimationList *animList;
	Ogre::Animation *cameraAnimation;
	Ogre::AnimationState *cameraAnimationState;
	double x, y, z;
	double x_offset, y_offset, z_offset;
	double fi, teta;
	double r;
	int i, j, k;
	double stepSize;
	double rotationAngle;
	double rotationStepSize;
	double tolerance;
	math::Pose newPos;
	math::Matrix3 yRotationMatrix;
	math::Matrix3 xRotationMatrix;
	math::Matrix3 pointToRotateMatrix;
	math::Vector3 columnVect;
	math::Vector3 zeroVect;
	Ogre::Vector3 posToUse;
	math::Matrix3 resultMatrix;
	math::Matrix3 testMatrix1;
	math::Matrix3 testMatrix2;
	event::ConnectionPtr updateConnection;
	transport::NodePtr senderNode;
	transport::PublisherPtr publisher;
	custom_pose_message::msgs::CustomPoseRequest msgToSend;
	bool called;
	bool present;
	bool anim;
	bool published;
	bool added;


	ros::ServiceServer service;
	ros::Subscriber sub;
	ros::AsyncSpinner *spinner;

};

}
