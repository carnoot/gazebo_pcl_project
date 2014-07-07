#ifndef _ROS_PLUGIN_SERVICE_HH
#define _ROS_PLUGIN_SERVICE_HH

#include "ros/ros.h"
#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo_pkg/AddTwoInts2.h"

namespace gazebo {

/// \Bla bla

class RosPluginService : public WorldPlugin
{

public: RosPluginService();

//public : ~RosPlugin();

	/// \The Load Method
public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

public: bool add(gazebo_pkg::AddTwoInts2::Request  &req,
         gazebo_pkg::AddTwoInts2::Response &res);

//public:
//	void OnUpdate(const common::UpdateInfo & /*_info*/);

public: ros::AsyncSpinner *spinner;

public: ros::ServiceServer service;

};

}

#endif
