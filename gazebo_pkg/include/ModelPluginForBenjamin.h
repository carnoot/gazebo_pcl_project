#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <gazebo.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <stdio.h>
#include "ros/ros.h"
#include "SherpaWorldObject.h"

namespace gazebo {

//typedef const boost::shared_ptr<geometry_msgs::Pose> customPosePtr;

class SherpaObjectModelPlugin: public ModelPlugin {

public:
	SherpaObjectModelPlugin();

public:
	virtual ~SherpaObjectModelPlugin();

public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

public:
	void CreateSherpaWorldObjectMessage();
	void InitService();

// Pointer to the model
private:

	gazebo_pkg::SherpaWorldObject mySherpaWorldObject;

	physics::ModelPtr my_model;
	sdf::ElementPtr my_sdf;

	math::Box objectBoundingBox;
	math::Pose objectPose;

	std::string objectTypeFieldString;
	std::string objectTopicFieldString;
	std::string objectType;
	std::string topicName;
	std::string objectName;

	ros::NodeHandle my_node;
	ros::Publisher my_publisher;

public:
	event::ConnectionPtr updateConnection;
	transport::NodePtr receiveNode;
	transport::SubscriberPtr subscriber;

};

}
