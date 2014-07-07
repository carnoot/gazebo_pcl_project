#include "model_push.h"

using namespace gazebo;

/////////////////////////////////////////////////
void ModelPush::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
	// Store the pointer to the model
	this->model = _parent;

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&ModelPush::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void ModelPush::OnUpdate(const common::UpdateInfo & /*_info*/) {
	// Apply a small linear velocity to the model.
	this->model->SetLinearVel(math::Vector3(.03, 0, 0));
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)

