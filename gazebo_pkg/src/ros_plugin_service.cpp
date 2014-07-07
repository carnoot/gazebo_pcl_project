#include "ros_plugin_service.h"

using namespace gazebo;

RosPluginService::RosPluginService() {
	int argc = 0;
	char ** argv = NULL;

	ros::init(argc, argv, "add_two_ints_server");

	ros::NodeHandle n;
	this->service = n.advertiseService("add_two_ints",&RosPluginService::add,this);

	ROS_INFO("Elindult!");

}

bool RosPluginService::add(gazebo_pkg::AddTwoInts2::Request  &req,
        gazebo_pkg::AddTwoInts2::Response &res) {
	res.sum = req.a + req.b;
	ROS_INFO("request: x=%ld, y=%ld", (long int )req.a, (long int )req.b);
	ROS_INFO("sending back response: [%ld]", (long int )res.sum);
	return true;
}

void RosPluginService::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

	this->spinner = new ros::AsyncSpinner(1);
	this->spinner->start();

	ROS_INFO("spinning");

}

//void RosPluginService::OnUpdate(const common::UpdateInfo & /*_info*/) {
//	// Apply a small linear velocity to the model.
//
//	ros::spin();
//
//}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(RosPluginService)
