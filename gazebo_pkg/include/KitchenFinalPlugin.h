#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <gazebo.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "/home/furdek/catkin_ws/build/gazebo_pkg/custom_pose_message/msgs/custom_pose_request.pb.h"
#include <stdio.h>
#include "ros/ros.h"
#include "/opt/ros/hydro/include/tf/transform_listener.h"

#include "/opt/ros/hydro/include/kdl_parser/kdl_parser.hpp"
#include <robot_state_publisher/robot_state_publisher.h>
#include "/opt/ros/hydro/include/urdf/model.h"
#include "/opt/ros/hydro/include/kdl/tree.hpp"

namespace gazebo {

//typedef const boost::shared_ptr<geometry_msgs::Pose> customPosePtr;

class KitchenFinal: public ModelPlugin {

public:
	KitchenFinal();

public:
	virtual ~KitchenFinal();

public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

public:
	void OnUpdate();

// Pointer to the model
private:

	std::string urdfPathName;
	std::string tf_prefix;

	KDL::Tree my_tree;
	const KDL::Tree* my_final_tree;
	urdf::Model my_model;
	robot_state_publisher::RobotStatePublisher* robotStatePublisher;
	std::map<std::string, double> joint_positions;

public:
	event::ConnectionPtr updateConnection;
	transport::NodePtr receiveNode;
	transport::SubscriberPtr subscriber;

};

}
