#ifndef _WORLD_EDIT_HH
#define _WORLD_EDIT_HH

#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo {

/// \Bla bla

class WorldEdit : public WorldPlugin
{
	/// \The Load Method
public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

};

}

#endif
