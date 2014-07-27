#ifndef _ROS_GET_COORD_HH
#define _ROS_GET_COORD_HH

#include "ros/ros.h"
#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo_pkg/GetObject.h"
#include "gazebo_pkg/Object.h"
#include "gazebo_pkg/ObjectInspectionNumber.h"
#include "gazebo_pkg/ObjectInspectionCenter.h"
#include "gazebo_pkg/ObjectInspectionBounding.h"
#include "gazebo_pkg/ObjectInspectionCameraPos.h"
#include "gazebo_pkg/ObjectInspectionStart.h"
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
			gazebo_pkg::GetObject::Response &res);bool ObjectToInspect(
			gazebo_pkg::ObjectInspectionNumber::Request &,
			gazebo_pkg::ObjectInspectionNumber::Response &);bool PassObjectCenter(
			gazebo_pkg::ObjectInspectionStart::Request &,
			gazebo_pkg::ObjectInspectionStart::Response &);

public:
	std::string CreatePositionString(gazebo_pkg::Object);
	std::string CreateModelNameString(gazebo_pkg::Object);
	std::string CreateMaterialColor(gazebo_pkg::Object);
	std::string CreateOrientationString(gazebo_pkg::Object);
	std::string CreateGeometry(gazebo_pkg::Object);
	std::string CreateMesh(gazebo_pkg::Object);
	void CreateShape(gazebo_pkg::Object);
	std::string AddStaticAttribute(bool);
	std::vector<float> QuaternionToEuler(float, float, float, float);
	std::string ConvertFloat(float);bool PassCameraPosition(
			gazebo_pkg::ObjectInspectionCameraPos::Request &,
			gazebo_pkg::ObjectInspectionCameraPos::Response &);
	void OnUpdate();

public:
	ros::AsyncSpinner *spinner;

public:
	ros::ServiceServer service;
	ros::ServiceServer service_1;
	ros::ServiceServer service_2;
	ros::ServiceServer service_3;
	event::ConnectionPtr updateConnection;

	std::pair<int, std::string> object_pair;
	std::vector<std::pair<int, std::string>> pair_values;

private:

	float camera_pos_x;
	float camera_pos_y;
	float camera_pos_z;
	float process_offset;

	math::Box bounding_box;
	std::vector<float> object_center;bool primit_coord;

private:
	physics::WorldPtr parent;

};

}

#endif
