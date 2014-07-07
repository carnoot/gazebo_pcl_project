/*
 * factory.cpp
 *
 *  Created on: Mar 12, 2014
 *      Author: furdek
 */

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo {
class Factory: public WorldPlugin {
public:
	void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
		// Option 1: Insert model from file via function call.
		// The filename must be in the GAZEBO_MODEL_PATH environment variable.
		_parent->InsertModelFile("model://first_test");
	}
};
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}
