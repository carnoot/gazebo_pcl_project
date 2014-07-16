#include "system_gui.h"

using namespace gazebo;


SystemGUI::SystemGUI(){

}

SystemGUI::~SystemGUI() {
	if (this->userCam)
		this->userCam->EnableSaveFrame(false);
}

void SystemGUI::Load(int /*_argc*/, char ** /*_argv*/) {
	std::cout << "LOAD" << std::endl;
	this->contor = 0;

	this->updateConnection = event::Events::ConnectPreRender(
				boost::bind(&SystemGUI::OnUpdate, this));

}

void SystemGUI::Init() {
	// Get a pointer to the active user camera
	this->userCam = gui::get_active_camera();

	// Enable saving frames
	this->userCam->EnableSaveFrame(true);

}

void SystemGUI::OnUpdate() {

	std::cout << "UPDATE" << std::endl;
	this->contor++;
	std::cerr << this->contor << std::endl;

}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
