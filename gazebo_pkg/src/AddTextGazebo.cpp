#include "AddTextGazebo.h"

using namespace gazebo;
using namespace std;
using namespace CEGUI;

AddTextGazebo::AddTextGazebo() {

	ROS_INFO("faszConstruct");

	this->numberOfCalls = 0;

	//this->initService();
	//this->initSubscriber();

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
//	this->scene = this->userCam->GetScene();
//	this->sceneManager = this->scene->GetManager();
//
//	math::Vector2d *windowPos = new math::Vector2d(50,50);
//	math::Vector2d *windowSize = new math::Vector2d(50,50);

//this->mainoverlay->CreateWindow("DefaultWindow","window1","wind",*windowPos,*windowSize,"FASZ");
//this->mainoverlay->Update();

//	this->belowVect = new math::Vector3(-2.6, -6.3, 5);
//
//	Ogre::Vector3 vect;
//	vect.x = 0;
//	vect.y = 0;
//	vect.z = 0;
//	Ogre::Camera *cam = this->sceneManager->createCamera("TestCam");
//	this->childNode =
//			this->sceneManager->getRootSceneNode()->createChildSceneNode(
//					"CamNode", vect,
//					this->sceneManager->getRootSceneNode()->getOrientation());
//	this->childNode->attachObject(cam);
//	this->childNode->setVisible(true);
//
//	std::cout << "BEFORE CHILDNODE2" << std::endl;
//
//	this->childNode2 = this->childNode->createChildSceneNode("CamNodeChild",
//			vect, this->childNode->getOrientation());
//
//	std::cout << "CHILD2 CREATED" << std::endl;
//
//	this->ent = this->sceneManager->createEntity("Sphere",
//			Ogre::SceneManager::PT_CUBE);
//
//	std::cout << "ENTITY CREATED" << std::endl;
//
//	this->childNode2->attachObject(this->ent);
//	this->childNode2->setVisible(true);
//
//	std::cout << this->userCam->GetSceneNode()->getName() << std::endl;

}

void AddTextGazebo::OnUpdate() {

//	    this->scene->GetVisualsBelowPoint(*belowVect, this->visualVect);
//
//	    std::cout << "VECT SIZE" << visualVect.size() << std::endl;
//
//	    for (int i = 0; i < this->visualVect.size(); ++i) {
//	        std::cout << this->visualVect[i]->GetName() << std::endl;
//	    }

	this->numberOfCalls++;
	std::cout << this->numberOfCalls << std::endl;

	if (this->numberOfCalls == 20) {

		Real panelPos[2];
		Real elementPos[2];
		Real panelDim[2];
		Real elementDim[2];
		std::string fontName = "Arial";
		std::string text = "EGY TEXT";
		float fontSize = 24;
		common::Color color;

		color.r = 1;
		color.g = 0.6;
		color.b = 0.6;
		color.a = 0.6;

		panelDim[0] = 300;
		panelDim[1] = 300;
		elementDim[0] = 100;
		elementDim[1] = 100;
		panelPos[0] = 160;
		panelPos[1] = 80;
		elementPos[0] = 10;
		elementPos[1] = 20;

		std::cout << "CREATE" << std::endl;
		AddTextOverlay(panelPos, panelDim, elementPos, elementDim, fontName,
				fontSize, color, text);
	}

//	if (this->called == true) {
//		AddTextGazebo::customCreateText(this->textRequest);
//		this->called = false;
//	}

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

//	AddTextGazebo::customCreateText(*message);

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

void AddTextGazebo::windowTest() {
	math::Vector2d *windowPos = new math::Vector2d(500, 500);
	math::Vector2d *windowSize = new math::Vector2d(850, 850);
	CEGUI::Window * myRoot =
			CEGUI::WindowManager::getSingleton().loadWindowLayout(
					"test.layout");
	System::getSingleton().setGUISheet(myRoot);

	CEGUI::Window* mEditorGuiSheet =
			CEGUI::WindowManager::getSingleton().createWindow(
					(CEGUI::utf8*) "DefaultWindow", (CEGUI::utf8*) "Sheet");
	System::getSingleton().setGUISheet(mEditorGuiSheet);
	CEGUI::Window* myWindow = CEGUI::WindowManager::getSingleton().getWindow(
			"root");
	myWindow->addChildWindow(myRoot);
	CEGUI::WindowManager::getSingleton().getWindow("mata")->setMousePassThroughEnabled(
			true);
	this->mainoverlay->CreateWindow("DefaultWindow", "window1", "root",
			*windowPos, *windowSize, "FASZ");
	System::getSingleton().setGUISheet(
			WindowManager::getSingleton().getWindow("window1"));

	this->mainoverlay->Hide();
	this->mainoverlay->Update();

	CEGUI::UDim windowDimension;
	windowDimension =
			CEGUI::WindowManager::getSingletonPtr()->getWindow("root")->getHeight();
	std::cout << "HEIGHT OFFSET:" << windowDimension.d_offset << std::endl;
	std::cout << "SCALE:" << windowDimension.d_scale << std::endl;
	CEGUI::WindowManager::getSingletonPtr()->getWindow("root")->getWidth();
	std::cout << "WIDTH OFFSET:" << windowDimension.d_offset << std::endl;
	std::cout << "SCALE:" << windowDimension.asAbsolute(1) << std::endl;
	std::cout << "RELATIVE" << windowDimension.asRelative(1) << std::endl;

}

void AddTextGazebo::KinectTest() {

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

	Real panelPos[2];
	Real elementPos[2];
	Real panelDim[2];
	Real elementDim[2];
	std::string fontName = "Arial";
	std::string text = "MEGY";
	float fontSize = 24;
	common::Color color;

	color.r = 1;
	color.g = 0.6;
	color.b = 0.6;

	panelDim[0] = 300;
	panelDim[1] = 300;
	elementDim[0] = 100;
	elementDim[1] = 100;
	panelPos[0] = 160;
	panelPos[1] = 80;
	elementPos[0] = 10;
	elementPos[1] = 20;

	std::cout << this->numberOfCalls << std::endl;

	if (this->numberOfCalls == 1) {
		std::cout << "CREATE" << std::endl;
		AddTextOverlay(panelPos, panelDim, elementPos, elementDim, fontName,
				fontSize, color, text);
	}
//	if (this->numberOfCalls == 2) {
	else {
		std::cout << "MODIFY" << std::endl;
		ModifyTextOverlay("FASZKI");
	}
	if (this->numberOfCalls == 3) {
		std::cout << "DELETE" << std::endl;
		DeleteTextOverlay();
	}

//	KinectTest();

//	textColor = new common::Color(textParam.customColor[0],
//			textParam.customColor[1], textParam.customColor[2],
//			textParam.customColor[3]);
//
//	std::string textName;
//
//	textName = "text";
//
//	textName.append(AddTextGazebo::convertFloat(this->numberOfCalls));
//
//	std::cout << textName << std::endl;
//	std::cout << textParam.customText << std::endl;
//	std::cout << textParam.customFont << std::endl;
//	std::cout << textParam.customFontSize << std::endl;
//	std::cout << textParam.customColor[0] << textParam.customColor[1]
//			<< textParam.customColor[2] << textParam.customColor[3]
//			<< std::endl;
//
//	text->Load(textName, textParam.customText, textParam.customFont,
//			textParam.customFontSize, *textColor);
//	text->setVisible(true);
//
//	this->visual = this->scene->GetVisual(textParam.object);
//	this->visual->Update();
//
//	std::cout << "WORLD" << this->visual->GetWorldPose() << std::endl;
//
//	std::cout << this->visual->GetName() << std::endl;
//
//	boundingBox = this->visual->GetBoundingBox();
//	boxCenter = boundingBox.GetCenter();
//
//	boxOgreCenter.x = this->visual->GetWorldPose().pos.x;
//	boxOgreCenter.y = this->visual->GetWorldPose().pos.y;
//	boxOgreCenter.z = this->visual->GetWorldPose().pos.z;
//
//	std::cout << "MAX" << boundingBox.max << std::endl;
//
//	std::cout << "MIN" << boundingBox.min << std::endl;

//	std::cout << "MAX" << boundingBox.max.x << std::endl;
//	std::cout << boundingBox.max.y << std::endl;
//	std::cout << boundingBox.max.z << std::endl;
//
//	std::cout << "MIN" << boundingBox.min.x << std::endl;
//	std::cout << boundingBox.min.y << std::endl;
//	std::cout << boundingBox.min.z << std::endl;

//	Ogre::Vector3 vect;
//	vect.x = 20;
//	vect.y = 50;
//	vect.z = 20;
//	Ogre::Camera *cam = this->sceneManager->createCamera("TestCam");
//	this->childNode = this->sceneManager->getRootSceneNode()->createChildSceneNode("CamNode",vect,this->sceneManager->getRootSceneNode()->getOrientation());
//	this->childNode->attachObject(cam);
//	this->childNode->setVisible(true);
//
//	std::cout << "BEFORE CHILDNODE2" << std::endl;
//
//	this->childNode2 = this->childNode->createChildSceneNode("CamNodeChild",vect,this->childNode->getOrientation());
//
//	std::cout << "CHILD2 CREATED" << std::endl;
//
//	this->ent = this->sceneManager->createEntity("Sphere", Ogre::SceneManager::PT_CUBE);
//
//	std::cout << "ENTITY CREATED" << std::endl;
//
//	this->childNode2->attachObject(this->ent);
//	this->childNode2->setVisible(true);

//	std::cout << "CAMERA INITIAL DER POS"
//			<< this->childNode->_getDerivedPosition() << std::endl;
//
//	std::cout << "CAMERA INITIAL POS" << this->childNode->getPosition()
//			<< std::endl;
//
//	std::cout << "CHILD INITIAL POS" << this->childNode2->getPosition()
//			<< std::endl;
//
//	std::cout << "CHILD INITIAL DER POS"
//			<< this->childNode2->_getDerivedPosition() << std::endl;
//
//	Ogre::Vector3 vect;
//	vect.x = 50;
//	vect.y = 50;
//	vect.z = 50;
//
//	this->childNode->translate(vect, Ogre::Node::TS_WORLD);
//
//	std::cout << "TRANSLATE 1 NODE1 CAMERA POS"
//			<< this->childNode->getPosition() << std::endl;
//
//	std::cout << "TRANSLATE 1 NODE1 CAMERA DER POS"
//			<< this->childNode->_getDerivedPosition() << std::endl;
//
//	std::cout << "TRANSLATE 1 NODE1 CHILD POS"
//			<< this->childNode2->getPosition() << std::endl;
//
//	std::cout << "TRANSLATE 1 NODE1 CHILD DER POS"
//			<< this->childNode2->_getDerivedPosition() << std::endl;
//
//	this->childNode2->translate(vect, Ogre::Node::TS_WORLD);

//	std::cout << "TRANSLATE 2 NODE2 CAMERA POS"
//			<< this->childNode->getPosition() << std::endl;
//
//	std::cout << "TRANSLATE 2 NODE2 CAMERA DER POS"
//			<< this->childNode->_getDerivedPosition() << std::endl;
//
//	std::cout << "TRANSLATE 2 NODE2 CHILD POS"
//			<< this->childNode2->getPosition() << std::endl;
//
//	std::cout << "TRANSLATE 2 NODE2 CHILD DER POS"
//			<< this->childNode2->_getDerivedPosition() << std::endl;

//	cameraVect.x = 0;
//	cameraVect.y = 0;
//	cameraVect.z = -2;

//	cameraVect.x = this->userCam->GetSceneNode()->getPosition().x;
//	cameraVect.y = this->userCam->GetSceneNode()->getPosition().y;
//	cameraVect.z = this->userCam->GetSceneNode()->getPosition().z;

//	this->childNode = this->userCam->GetSceneNode()->createChildSceneNode(
//			cameraVect, this->userCam->GetSceneNode()->getOrientation());
//	this->childNode->setPosition(0, 0, -2);
//
//	this->childNode->attachObject(text);
//	this->childNode->setVisible(true);

//	this->node = this->visual->GetSceneNode();
//	this->node->setVisible(true);
//
//	this->childNode = this->node->createChildSceneNode(boxOgreCenter,
//			this->node->getOrientation());
//	this->childNode->setPosition(3, 3, 3);
//
//	this->childNode->setPosition(this->visual->GetWorldPose().pos.x, this->visual->GetWorldPose().pos.y, this->visual->GetWorldPose().pos.z);
//
//	this->childNode->setPosition(boundingBox.GetCenter().x,
//			boundingBox.GetCenter().y,
//			boundingBox.GetCenter().z + boundingBox.GetZLength()
//					+ textParam.customFontSize + 0.25);
//
//	this->childNode->attachObject(text);
//	this->childNode->setVisible(true);

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
