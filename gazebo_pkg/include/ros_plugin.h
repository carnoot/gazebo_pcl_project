#ifndef _ROS_PLUGIN_HH
#define _ROS_PLUGIN_HH

#include "gazebo/gazebo.hh"

namespace gazebo {

/// \Bla bla

class RosPlugin : public WorldPlugin
{

public: RosPlugin();

//public : ~RosPlugin();

	/// \The Load Method
public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
		void OnUpdate();

public: event::ConnectionPtr updateConnection;
		physics::WorldPtr parent;

};

}

#endif
