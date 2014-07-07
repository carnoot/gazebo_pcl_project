#include "DrawLinesPlugin.h"

using namespace gazebo;

DrawLinesPlugin::DrawLinesPlugin() {

}

DrawLinesPlugin::~DrawLinesPlugin() {

}

void DrawLinesPlugin::Load(int /*_argc*/, char ** /*_argv*/) {

}

void DrawLinesPlugin::Init() {

	float a = 1;
	float b = 1;
	float c = 1;

	this->userCam = gui::get_active_camera();
	this->scene = this->userCam->GetScene();

	this->vectstart = new math::Vector3(a-3, b-2, c);

	this->vectend = new math::Vector3(a + 2, b + 5, c + 2);

	this->scene->DrawLine(*this->vectstart, *this->vectend, "geci");

	this->vectstart = new math::Vector3(a, b, c);

	this->vectend = new math::Vector3(a + 2, b + 2, c + 2);

	this->userCam->GetScene()->DrawLine(*this->vectstart, *this->vectend,
			"fasz");

}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(DrawLinesPlugin)
