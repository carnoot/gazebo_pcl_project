#include "KitchenFinalPlugin.h"

using namespace gazebo;

KitchenFinal::KitchenFinal() {

	this->urdfPathName =
			"/home/furdek/catkin_ws/src/gazebo_pkg/models/kitchen_final/kitchen_area.urdf";
	this->tf_prefix = "my_prefix";

	this->joint_positions.insert(
			std::map<std::string, double>::value_type(
					"island_block_stove_joint", 1));
	this->joint_positions.insert(
			std::map<std::string, double>::value_type(
					"island_block_counter_side_island_right_joint", 2));

//	this->robotStatePublisher(this->my_tree);

}

KitchenFinal::~KitchenFinal() {
}

void KitchenFinal::Load(physics::ModelPtr _parent, sdf::ElementPtr _element) {

	ros::Time::init();

	std::cout << "KitchenPlugin Load 2 " << std::endl;

	if (!this->my_model.initFile(this->urdfPathName)) {
		ROS_ERROR("Failed to parse urdf robot model");
	}
	if (!kdl_parser::treeFromUrdfModel(this->my_model, this->my_tree)) {
		ROS_ERROR("Failed to construct kdl tree");
	}

	this->my_final_tree = &this->my_tree;

//	this->my_final_tree = this->my_tree;

//	this->robotStatePublisher = new robot_state_publisher::RobotStatePublisher(*this->my_final_tree);

//	this->robotStatePublisher = new robot_state_publisher::RobotStatePublisher(
//			this->my_tree);

//	std::cout << "Number of joints in : " << this->my_tree.getNrOfJoints()
//			<< std::endl;

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&KitchenFinal::OnUpdate, this));

}

void KitchenFinal::OnUpdate() {

	std::cout << "ON Update" << std::endl;

//	this->robotStatePublisher->publishTransforms(this->joint_positions,
//			ros::Time::now(), this->tf_prefix);

	this->robotStatePublisher->publishFixedTransforms(this->tf_prefix);

//	if (!this->my_model.initFile(this->urdfPathName)) {
//		ROS_ERROR("Failed to parse urdf robot model");
//	}
//
//	if (!kdl_parser::treeFromUrdfModel(this->my_model, this->my_tree)) {
//		ROS_ERROR("Failed to construct kdl tree");
//	}
//	std::cout << "Number of joints: " << this->my_tree.getNrOfJoints()
//			<< std::endl;

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (KitchenFinal)
