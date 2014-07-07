#include "ros_plugin.h"
#include "ros/ros.h"
//#include "gazebo_pkg/AddTwoInts.h"

using namespace gazebo;

	RosPlugin::RosPlugin(){

		//ros::init(a, b, "clientul");

		//ros::NodeHandle n;
		//ros::ServiceClient client = n.serviceClient("client");

		ROS_INFO("Merge constructorul, asa ca e in regula!");

	}

    void RosPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {

    }

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(RosPlugin)
