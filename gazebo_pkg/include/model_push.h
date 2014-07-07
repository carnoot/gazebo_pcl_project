#ifndef _GAZEBO_MODEL_PUSH_HH_
#define _GAZEBO_MODEL_PUSH_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo {

/// \brief A plugin
class ModelPush: public ModelPlugin {
	/// \brief Load the plugin.
public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

	/// \OnUpdate
public:
	void OnUpdate(const common::UpdateInfo & /*_info*/);

	// Pointer to the model
private:
	physics::ModelPtr model;

	// Pointer to the update event connection
private:
	event::ConnectionPtr updateConnection;

};

}

#endif
