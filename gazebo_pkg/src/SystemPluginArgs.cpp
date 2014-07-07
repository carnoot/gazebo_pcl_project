#include "SystemPluginArgs.h"

using namespace gazebo;
using namespace std;
using namespace CEGUI;

AddTextGazebo::AddTextGazebo() {

	this->numberOfCalls = 0;

	this->initService();
	//this->initSubscriber();

	this->updateConnection = event::Events::ConnectPreRender(
			boost::bind(&AddTextGazebo::OnUpdate, this));

}

AddTextGazebo::~AddTextGazebo() {

}

void AddTextGazebo::Load(int argc, char ** argv) {

	std::cout << "LOAD" << std::endl;

	std::cout << "Number of parameters:" << argc << std::endl;

	std::cout << "First param:" << argv[0] << std::endl;

}

void AddTextGazebo::Init() {

	this->userCam = gui::get_active_camera();
	this->mainoverlay = this->userCam->GetGUIOverlay();
	this->scene = this->userCam->GetScene();
	this->sceneManager = this->scene->GetManager();

}

void AddTextGazebo::OnUpdate() {

	if (this->called == true) {
		AddTextGazebo::customCreateText(this->textRequest);
		this->called = false;
	}

}

void AddTextGazebo::initSubscriber() {

	int argc = 0;
	char ** argv = NULL;

	ros::init(argc, argv, "listener");

	ros::NodeHandle n;
	ros::SubscribeOptions options;
	this->sub = n.subscribe("publisher", 1000, &AddTextGazebo::callback, this);

	this->spinner = new ros::AsyncSpinner(1);
	this->spinner->start();

	ROS_INFO("Waiting for text parameters PUBLISHER!");

}

void AddTextGazebo::callback(
		const gazebo_pkg::GetTextParam::Request::ConstPtr& message) {

	std::cout << "Publisher CallBack" << std::endl;

	this->textRequest = *message;
	this->numberOfCalls++;
	this->called = true;


}

void AddTextGazebo::initService() {

	int argc = 0;
	char ** argv = NULL;

	ros::init(argc, argv, "get_text_server");

	ros::NodeHandle n;
	this->service = n.advertiseService("get_text_param",
			&AddTextGazebo::getTextParam, this);

	this->spinner = new ros::AsyncSpinner(1);
	this->spinner->start();

	ROS_INFO("Waiting for text parameters SERVICE!");
}

bool AddTextGazebo::getTextParam(gazebo_pkg::GetTextParam::Request &req,
		gazebo_pkg::GetTextParam::Response &res) {

	this->numberOfCalls++;

	this->textRequest = req;

	this->called = true;

	return true;

}

void AddTextGazebo::customCreateText(
		gazebo_pkg::GetTextParam::Request textParam) {

}

void AddTextGazebo::delayFunction() {
	while (this->timer->getMilliseconds() < 3000) {

	}
}

std::string AddTextGazebo::convertFloat(float a) {
	std::ostringstream buff;
	buff << a;
	return buff.str();
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(AddTextGazebo)
