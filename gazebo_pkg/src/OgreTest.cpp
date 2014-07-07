#include "OgreTest.h"

using namespace gazebo;
using namespace Ogre;

OgreTest::OgreTest() {

	ROS_INFO("faszConstruct");

	this->numberOfCalls = 0;

//	this->initService();

	this->updateConnection = event::Events::ConnectPreRender(
			boost::bind(&OgreTest::OnUpdate, this));

}

OgreTest::~OgreTest() {

}

void OgreTest::Load(int /*_argc*/, char ** /*_argv*/) {

	std::cout << "LOAD" << std::endl;

}

void OgreTest::Init() {

	std::cout << "INIT" << std::endl;

	Ogre::Vector3 *pozvect = new Ogre::Vector3(1, 1, 1);
	Ogre::Quaternion *quat = new Ogre::Quaternion(1, 0, 0, 0);

	this->userCam = gui::get_active_camera();
	this->scene = this->userCam->GetScene();
	if (this->visual != NULL)
		std::cout << "VISUAL OK" << this->visual->GetName() << std::endl;
	this->sceneManager = this->scene->GetManager();
	this->ent = this->sceneManager->createEntity("sphere",
			Ogre::SceneManager::PT_SPHERE);

	int argc = 0;
	char ** argv = NULL;

	ResourceGroupManager::getSingletonPtr()->addResourceLocation("/home/furdek","FileSystem");
	ResourceGroupManager::getSingletonPtr()->initialiseAllResourceGroups();
	Entity* e = this->sceneManager->createEntity("stl1.mesh");
	SceneNode* my_node = this->sceneManager->getRootSceneNode()->createChildSceneNode("my_scene_node");
	my_node->setPosition(*pozvect);
	my_node->setVisible(true);
	my_node->attachObject(e);

	std::cout << "Entity && SceneNode created!" << std::endl;

}

void OgreTest::OnUpdate() {

	if (this->called == true) {
		OgreTest::customCreateText(this->textRequest);
		this->called = false;
	}

}

void OgreTest::initService() {

	int argc = 0;
	char ** argv = NULL;

	ros::init(argc, argv, "get_text_server");

	ros::NodeHandle n;
	this->service = n.advertiseService("get_text_param",
			&OgreTest::getTextParam, this);

	this->spinner = new ros::AsyncSpinner(1);
	this->spinner->start();

	ROS_INFO("Waiting for text parameters!");
}

bool OgreTest::getTextParam(gazebo_pkg::GetTextParam::Request &req,
		gazebo_pkg::GetTextParam::Response &res) {

	this->textRequest = req;

	this->called = true;
	this->numberOfCalls++;

	return true;

}

void OgreTest::customCreateText(gazebo_pkg::GetTextParam::Request textParam) {

	rendering::MovableText* text = new rendering::MovableText();
	math::Box boundingBox;
	math::Vector3 boxCenter;
	Ogre::Vector3 boxOgreCenter;
	common::Color *textColor;
	Ogre::MovableObject *obj;

	textColor = new common::Color(textParam.customColor[0],
			textParam.customColor[1], textParam.customColor[2],
			textParam.customColor[3]);

	std::string textName;

	textName = "text";

	textName.append(OgreTest::convertFloat(this->numberOfCalls));

	std::cout << textName << std::endl;
	std::cout << textParam.customText << std::endl;
	std::cout << textParam.customFont << std::endl;
	std::cout << textParam.customFontSize << std::endl;
	std::cout << textParam.customColor[0] << textParam.customColor[1]
			<< textParam.customColor[2] << textParam.customColor[3]
			<< std::endl;

	text->Load(textName, textParam.customText, textParam.customFont,
			textParam.customFontSize, *textColor);
	text->setVisible(true);

	this->visual = this->scene->GetVisual(textParam.object);
	this->visual->Update();

	std::cout << "WORLD" << this->visual->GetWorldPose() << std::endl;

	std::cout << this->visual->GetName() << std::endl;

	boundingBox = this->visual->GetBoundingBox();
	boxCenter = boundingBox.GetCenter();

	boxOgreCenter.x = (Real) boxCenter.x;
	boxOgreCenter.y = (Real) boxCenter.y;
	boxOgreCenter.z = (Real) boxCenter.z;

	std::cout << "MAX" << boundingBox.max.x << std::endl;
	std::cout << boundingBox.max.y << std::endl;
	std::cout << boundingBox.max.z << std::endl;

	std::cout << "MIN" << boundingBox.min.x << std::endl;
	std::cout << boundingBox.min.y << std::endl;
	std::cout << boundingBox.min.z << std::endl;

	std::cout << boundingBox.GetZLength() << std::endl;

	this->node = this->visual->GetSceneNode();
	this->node->setVisible(true);

	this->childNode = this->node->createChildSceneNode(boxOgreCenter,
			this->node->getOrientation());
	this->childNode->setPosition(3, 3, 3);
	this->childNode->setPosition(boundingBox.GetCenter().x,
			boundingBox.GetCenter().y,
			boundingBox.GetCenter().z + boundingBox.GetZLength()
					+ textParam.customFontSize + 0.25);

	this->childNode->attachObject(text);
	this->childNode->setVisible(true);

}

void OgreTest::delayFunction() {
	while (this->timer->getMilliseconds() < 3000) {

	}
}

std::string OgreTest::convertFloat(float a) {
	std::ostringstream buff;
	buff << a;
	return buff.str();
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(OgreTest)
