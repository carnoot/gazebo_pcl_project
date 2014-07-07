#ifndef _VisualPluginTest_HH_
#define _VisualPluginTest_HH_

#include <map>
#include <vector>
#include <string>

#include "gazebo/math/Pose.hh"
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <rendering/Visual.hh>

namespace gazebo {

class VisualPluginTest: public VisualPlugin {

public:
	void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);
	VisualPluginTest();
	~VisualPluginTest();

private:
	rendering::VisualPtr visual;
	physics::WorldPtr world;
	physics::ModelPtr model1;
};
}

#endif
