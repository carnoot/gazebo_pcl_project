#ifndef _ROS_GET_COORD_HH
#define _ROS_GET_COORD_HH

#include "ros/ros.h"
#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo_pkg/GetObject.h"
#include "gazebo_pkg/Object.h"
#include <cstdlib>
#include <sstream>
#include <vector>

namespace gazebo {

/// \Bla bla

class RosGetCoord: public WorldPlugin {

public:
	RosGetCoord();

//public : ~RosPlugin();

/// \The Load Method
public:
	void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

//public:
//	bool GetCoord(gazebo_pkg::GetCoordFromClient::Request &req,
//			gazebo_pkg::GetCoordFromClient::Response &res);

public:
	bool GetObjects(gazebo_pkg::GetObject::Request &req,
			gazebo_pkg::GetObject::Response &res);

public:
	std::string CreatePositionString(gazebo_pkg::Object obj);
	std::string CreateModelNameString(gazebo_pkg::Object obj);
	std::string CreateMaterialColor(gazebo_pkg::Object obj);
	std::string CreateOrientationString(gazebo_pkg::Object obj);
	std::string CreateGeometry(gazebo_pkg::Object obj);
	void CreateShape(gazebo_pkg::Object);
	std::vector<float> QuaternionToEuler(float w, float x,
			float y, float z);
	std::string ConvertFloat(float a);

public:
	ros::AsyncSpinner *spinner;

public:
	ros::ServiceServer service;

private:
	bool primit_coord;

private:
	physics::WorldPtr parent;

};

}

#endif
