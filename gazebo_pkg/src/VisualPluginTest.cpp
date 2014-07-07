#include "VisualPluginTest.h"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////

VisualPluginTest::VisualPluginTest() {

}

VisualPluginTest::~VisualPluginTest(){

}

void VisualPluginTest::Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/) {

	this->visual = _parent;

	this->visual->SetMaterial("Ias/KitchenSink");

}

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(VisualPluginTest)

