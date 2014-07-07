#include "DrawTextPlugin.h"

using namespace gazebo;
using namespace Ogre;

DrawTextPlugin::DrawTextPlugin() {

	ROS_INFO("faszConstruct");

}

DrawTextPlugin::~DrawTextPlugin() {

}

void DrawTextPlugin::Load(int /*_argc*/, char ** /*_argv*/) {

}

void DrawTextPlugin::Init() {

//	if (gui::get_active_camera()->GetScene()->GetManager() == NULL){
//		std::cout << "manager is null";
//	}

	float a=1;
	float b=1;
	float c=2;

	this->userCam = gui::get_active_camera();

	this->scene = this->userCam->GetScene();

	this->sceneManager = this->scene->GetManager();

	std::cout << this->sceneManager->getName() << std::endl;

	std::cout << "GADFGAF2" << std::endl;

		this->vectstart = new math::Vector3(a-3, b-2, c);

		this->vectend = new math::Vector3(a + 2, b + 5, c + 2);

		this->scene->DrawLine(*this->vectstart, *this->vectend, "geci");

	std::cout << "GADFGAF3333" << std::endl;

	ManualObject *obj = this->sceneManager->createManualObject("helo");

	std::cout << "GADFGAF44444" << std::endl;

	obj->clear();
	obj->begin("Gazebo/Red", Ogre::RenderOperation::OT_LINE_LIST);
	obj->position(1, 1, 1);
	obj->position(2, 2, 2);
	obj->end();

	node->attachObject(obj);

//	MovableObject* mobj = (MovableObject*) this->sceneManager->createEntity(
//			"Plane", SceneManager::PT_SPHERE);

//	Entity* planeEnt = this->sceneManager->createEntity("Plane", SceneManager::PT_SPHERE);
	std::cout << "GADFGAF2";
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(DrawTextPlugin)
