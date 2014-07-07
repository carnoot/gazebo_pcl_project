#ifndef _DynamicJoint_HH_
#define _DynamicJoint_HH_

#include <map>
#include <vector>
#include <string>

#include "gazebo/math/Pose.hh"
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class DynamicJoint: public WorldPlugin {

public:
	void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
	DynamicJoint();

private:
	physics::WorldPtr world;
	physics::ModelPtr model1;
	physics::ModelPtr model2;
	physics::JointPtr createdJoint;
	physics::PhysicsEnginePtr physics;
	physics::LinkPtr link1;
	physics::LinkPtr link2;

};
}

#endif
