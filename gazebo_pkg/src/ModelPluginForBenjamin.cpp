#include "ModelPluginForBenjamin.h"

using namespace gazebo;

SherpaObjectModelPlugin::SherpaObjectModelPlugin() {

	this->objectTypeFieldString = "ObjectType";
	this->objectTopicFieldString = "TopicName";

}

SherpaObjectModelPlugin::~SherpaObjectModelPlugin() {
}

void SherpaObjectModelPlugin::Load(physics::ModelPtr _parent,
		sdf::ElementPtr _element) {

	this->my_model = _parent;
	this->my_sdf = _element;

	this->InitService();

	if (!this->my_sdf->HasElement(this->objectTypeFieldString)) {
		ROS_ERROR("Field ObjectType hasn't been defined for this model!");
	} else {
		this->objectType =
				this->my_sdf->GetElement(this->objectTypeFieldString)->Get<
						std::string>();
	}

	if (!this->my_sdf->HasElement(this->objectTopicFieldString)) {
		ROS_ERROR("Field TopicName hasn't been defined for this model!");
		ROS_INFO("Default TopicName = SherpaWorldObjectTopic will be given!");
		this->topicName = "SherpaWorldObjectTopic";
	} else {
		this->objectType = this->my_sdf->GetElement(
				this->objectTopicFieldString)->Get<std::string>();
	}

//	this->objectBoundingBox = this->my_model->GetBoundingBox();
//	this->objectPose = this->my_model->GetWorldPose();
//	this->objectName = this->my_model->GetName();

	this->CreateSherpaWorldObjectMessage();
	this->my_publisher.publish(this->mySherpaWorldObject);

}

void SherpaObjectModelPlugin::CreateSherpaWorldObjectMessage() {

	this->mySherpaWorldObject.NAME = this->my_model->GetName();

	this->mySherpaWorldObject.TYPE = this->objectType;

	this->mySherpaWorldObject.POSE.position.x =
			this->my_model->GetWorldPose().pos.x;
	this->mySherpaWorldObject.POSE.position.y =
			this->my_model->GetWorldPose().pos.y;
	this->mySherpaWorldObject.POSE.position.z =
			this->my_model->GetWorldPose().pos.z;

	this->mySherpaWorldObject.POSE.orientation.w =
			this->my_model->GetWorldPose().rot.w;
	this->mySherpaWorldObject.POSE.orientation.x =
			this->my_model->GetWorldPose().rot.x;
	this->mySherpaWorldObject.POSE.orientation.y =
			this->my_model->GetWorldPose().rot.y;
	this->mySherpaWorldObject.POSE.orientation.z =
			this->my_model->GetWorldPose().rot.z;

//	this->mySherpaWorldObject.MIN.elems[0] =
//			(float) this->my_model->GetBoundingBox().min.x;
//	this->mySherpaWorldObject.MIN.elems[1] =
//			(float) this->my_model->GetBoundingBox().min.y;
//	this->mySherpaWorldObject.MIN.elems[2] =
//			(float) this->my_model->GetBoundingBox().min.z;
//
//	this->mySherpaWorldObject.MAX.elems[0] =
//			(float) this->my_model->GetBoundingBox().max.x;
//	this->mySherpaWorldObject.MAX.elems[1] =
//			(float) this->my_model->GetBoundingBox().max.y;
//	this->mySherpaWorldObject.MAX.elems[2] =
//			(float) this->my_model->GetBoundingBox().max.z;

}

void SherpaObjectModelPlugin::InitService() {

	int argc = 0;
	char **argv = NULL;

	ros::init(argc, argv, "SherpaPublisher");

	this->my_publisher = this->my_node.advertise<gazebo_pkg::SherpaWorldObject>(
			this->topicName, 1000);

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (SherpaObjectModelPlugin)
