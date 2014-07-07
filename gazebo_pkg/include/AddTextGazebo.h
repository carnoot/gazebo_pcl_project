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

using namespace Ogre;

namespace gazebo {

/// \SystemGui Class
class AddTextGazebo: public SystemPlugin {

	/// \ Constructor
public:
	AddTextGazebo();

public:
	virtual ~AddTextGazebo();

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
	void initSubscriber();
	void callback(const gazebo_pkg::GetTextParam::Request::ConstPtr& message);
	void AddTextOverlay(Real panelPosition[2], Real panelDimension[2],
			Real elementPosition[2], Real elementDimension[2],
			std::string elementFontName, float elementFontSize,
			common::Color color, std::string textToDisplay);
	void DeleteTextOverlay();
	void ModifyTextOverlay(std::string textToDisplay);
	void windowTest();
	void KinectTest();

public:
	rendering::ScenePtr scene;
	rendering::VisualPtr visual;
	rendering::GUIOverlay *mainoverlay;
	Ogre::SceneManager *sceneManager;
	Ogre::SceneNode *node;
	Ogre::SceneNode *childNode;
	Ogre::SceneNode *childNode2;
	Ogre::Entity *ent;
	ros::ServiceServer service;
	ros::Subscriber sub;
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

