#include "DynamicJoint.h"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////

DynamicJoint::DynamicJoint() {

}

void DynamicJoint::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {

	this->world = _parent;
	this->model1 = this->world->GetModel("box");
	this->model2 = this->world->GetModel("box1");

	this->physics = this->world->GetPhysicsEngine();

	this->link1 = this->model1->GetLink("link");
	this->link2 = this->model2->GetLink("link");

	math::Pose position;
	math::Angle angle;

	position.pos.x = 0;
	position.pos.y = 0;
	position.pos.z = 0;

	position.rot.w = 1;
	position.rot.x = 0;
	position.rot.y = 0;
	position.rot.z = 0;

	this->createdJoint = this->physics->CreateJoint("revolute", this->model1);
	this->createdJoint->Load(this->link1, this->link2, position);
	this->createdJoint->Init();
	this->createdJoint->SetHighStop(0, 0.3);
	this->createdJoint->SetLowStop(0, -0.3);

}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(DynamicJoint)

