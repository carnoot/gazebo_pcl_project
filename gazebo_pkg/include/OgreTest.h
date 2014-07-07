#include <boost/bind.hpp>

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include <gazebo/common/common.hh>
#include "gazebo/gazebo.hh"
#include "gazebo_pkg/GetTextParam.h"
#include "ros/ros.h"

namespace gazebo {

/// \SystemGui Class
class OgreTest: public SystemPlugin {

	/// \ Constructor
public:
	OgreTest();

public:
	virtual ~OgreTest();

/// \Load

public:
	virtual void Load(int /*_argc*/, char ** /*_argv*/);

/// \Init

private:
	virtual void Init();

public:
	virtual void OnUpdate();
	void delayFunction();
	void HandleKeyPressEvent(const std::string &_key);
	bool getTextParam(gazebo_pkg::GetTextParam::Request &req,
			gazebo_pkg::GetTextParam::Response &res);
public:
	void customCreateText(gazebo_pkg::GetTextParam::Request textParam);
	void initService();
	std::string convertFloat(float _float);

public:
	rendering::ScenePtr scene;
	rendering::VisualPtr visual;
	Ogre::SceneManager *sceneManager;
	Ogre::SceneNode *node;
	Ogre::SceneNode *childNode;
	Ogre::Entity *ent;
	ros::ServiceServer service;
	ros::AsyncSpinner *spinner;
	float numberOfCalls;

private:
	rendering::MovableText* text;
	rendering::UserCameraPtr userCam;
	gazebo::math::Vector3 *vectstart;
	math::Vector3 *vectend;
	event::ConnectionPtr updateConnection;
	Ogre::Timer *timer;
	event::ConnectionPtr mouseConnection;
	common::MouseEvent *mouse;
	math::Vector3 *belowVect;
	std::vector<rendering::VisualPtr> visualVect;
	gazebo_pkg::GetTextParam::Request textRequest;
	bool called;
};

}

