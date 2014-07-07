#include "AddTextGazebo.h"

using namespace gazebo;
using namespace std;
using namespace CEGUI;

AddTextGazebo::AddTextGazebo() {

	this->numberOfCalls = 0;

	this->initService();

	this->updateConnection = event::Events::ConnectPreRender(
			boost::bind(&AddTextGazebo::OnUpdate, this));

}

AddTextGazebo::~AddTextGazebo() {

}

void AddTextGazebo::Load(int /*_argc*/, char ** /*_argv*/) {

	std::cout << "LOAD" << std::endl;

}

void AddTextGazebo::Init() {

	this->userCam = gui::get_active_camera();
	this->mainoverlay = this->userCam->GetGUIOverlay();


}
//AddTextOverlay trebuie completat cu parametri
void AddTextGazebo::OnUpdate() {

	if (this->called == true) {
		//AddTextGazebo::AddTextOverlay();
		//AddTextGazebo::customCreateText(this->textRequest);
		this->called = false;
	}

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


/// pentru AddTextOverlay, parametrii acestei functii nu mai au nici un rol
bool AddTextGazebo::getTextParam(gazebo_pkg::GetTextParam::Request &req,
		gazebo_pkg::GetTextParam::Response &res) {

	this->numberOfCalls++;

	this->textRequest = req;

	this->called = true;

	return true;

}

void AddTextGazebo::AddTextOverlay(Real panelPosition[2],
		Real panelDimension[2], Real elementPosition[2],
		Real elementDimension[2], std::string elementFontName,
		float elementFontSize, common::Color color, std::string textToDisplay) {
	Ogre::OverlayContainer* panel =
			(Ogre::OverlayContainer*) OverlayManager::getSingletonPtr()->createOverlayElement(
					"Panel", "myPanel");

	Ogre::DisplayString displayString;
	std::string fontSize;
	std::string colorString;
	fontSize = convertFloat(elementFontSize);

	colorString.append(convertFloat(color.r));
	colorString.append(" ");
	colorString.append(convertFloat(color.g));
	colorString.append(" ");
	colorString.append(convertFloat(color.b));

	displayString = textToDisplay;

	panel->setMetricsMode(Ogre::GMM_PIXELS);
	panel->setPosition(panelPosition[0], panelPosition[1]);
	panel->setDimensions(panelDimension[0], panelDimension[1]);

	Ogre::OverlayElement *element =
			OverlayManager::getSingletonPtr()->createOverlayElement("TextArea",
					"myText");

	element->setMetricsMode(Ogre::GMM_PIXELS);
	element->setDimensions(elementDimension[0], elementDimension[1]);
	element->setPosition(elementPosition[0], elementPosition[1]);
	element->setParameter("font_name", elementFontName);
	element->setParameter("char_height", fontSize);
	element->setParameter("colour", colorString);
	element->setCaption(displayString);

	panel->addChild(element);

	Ogre::Overlay *overlay = OverlayManager::getSingletonPtr()->create(
			"myOverlay");
	overlay->add2D(panel);
	overlay->show();
}

void AddTextGazebo::DeleteTextOverlay() {

	OverlayManager::getSingletonPtr()->destroyAll();

}

void AddTextGazebo::ModifyTextOverlay(std::string textToDisplay) {

	Ogre::DisplayString displayString;
	displayString = textToDisplay;

	Ogre::OverlayManager::getSingletonPtr()->getOverlayElement("myText")->setCaption(
			displayString);

}



void AddTextGazebo::customCreateText(
		gazebo_pkg::GetTextParam::Request textParam) {

//	rendering::MovableText* text = new rendering::MovableText();
//	math::Box boundingBox;
//	math::Vector3 boxCenter;
//	Ogre::Vector3 boxOgreCenter;
//	Ogre::Vector3 cameraVect;
//	common::Color *textColor;
//	Ogre::MovableObject *obj;



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
