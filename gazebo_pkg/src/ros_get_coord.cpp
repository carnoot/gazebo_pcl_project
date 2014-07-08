#include "ros_get_coord.h"
#include "sstream"

using namespace gazebo;

RosGetCoord::RosGetCoord() {
	int argc = 0;
	char ** argv = NULL;

	ros::init(argc, argv, "get_object_server");

	ros::NodeHandle n;
	this->service = n.advertiseService("get_object", &RosGetCoord::GetObjects,
			this);

	ROS_INFO("Waiting for coordinates!");

}

void RosGetCoord::OnUpdate(){

//	std::cerr << "Model count: " << this->parent->GetModelCount() << std::endl;
//	this->parent->GetModel(2)->GetBoundingBox()

}

bool RosGetCoord::GetObjects(gazebo_pkg::GetObject::Request &req,
		gazebo_pkg::GetObject::Response &res) {

	std::vector<gazebo_pkg::Object> vect;
	gazebo_pkg::Object currentObject;

	vect = req.msg;

	for (std::vector<gazebo_pkg::Object>::iterator it = vect.begin();
			it != vect.end(); ++it) {

		currentObject = *it;

		this->CreateShape(currentObject);

//		ROS_INFO("ID = %d, COLOR = %s", it->ID, it->COLOR.c_str());

	}

	return true;
}

void RosGetCoord::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

	this->parent = _parent;

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&RosGetCoord::OnUpdate, this));

	this->spinner = new ros::AsyncSpinner(1);
	this->spinner->start();

	ROS_INFO("spinning");

}

std::string RosGetCoord::ConvertFloat(float a) {
	std::ostringstream buff;
	buff << a;
	return buff.str();
}

std::vector<float> RosGetCoord::QuaternionToEuler(float w, float x, float y,
		float z) {
	math::Quaternion quat = math::Quaternion(w, x, y, z);

	std::vector<float> localVector;

	localVector.push_back(quat.GetAsEuler().x);
	localVector.push_back(quat.GetAsEuler().y);
	localVector.push_back(quat.GetAsEuler().z);

	return localVector;

}

std::string RosGetCoord::CreatePositionString(gazebo_pkg::Object obj) {

	std::string localString;
	std::ostringstream converter;

	localString.append("<pose>");

	converter << obj.pose.position.x;

	localString.append(converter.str());
	localString.append(" ");

	converter.str("");

	converter << obj.pose.position.y;

	localString.append(converter.str());
	localString.append(" ");

	converter.str("");

	converter << obj.pose.position.z;

	localString.append(converter.str());
	localString.append(" ");

	return localString;

}

std::string RosGetCoord::CreateOrientationString(gazebo_pkg::Object obj) {

	std::string localString;
	std::ostringstream converter;

	std::vector<float> euler_angles = RosGetCoord::QuaternionToEuler(
			obj.pose.orientation.w, obj.pose.orientation.x,
			obj.pose.orientation.y, obj.pose.orientation.z);

	converter << euler_angles[0];

	localString.append(converter.str());
	localString.append(" ");

	converter.str("");

	converter << euler_angles[1];

	localString.append(converter.str());
	localString.append(" ");

	converter.str("");

	converter << euler_angles[2];
	localString.append(converter.str());
	localString.append("</pose>\n");

	return localString;

}

std::string RosGetCoord::CreateModelNameString(gazebo_pkg::Object obj) {

	std::string localString;
	std::ostringstream converter;

	localString.append("'");
	localString.append(obj.SHAPE);

	converter << obj.ID;

	localString.append(converter.str());
	localString.append("'>\\");

	return localString;

}

std::string RosGetCoord::CreateGeometry(gazebo_pkg::Object obj) {

	std::string localString;
	std::ostringstream converter;

	localString.append("<");

	localString.append(obj.SHAPE);

	localString.append(">");

	if (obj.SHAPE == "box") {

		localString.append("<size>");

		converter << obj.SIZE[0];

		localString.append(converter.str());
		localString.append(" ");

		converter.str("");

		converter << obj.SIZE[1];

		localString.append(converter.str());
		localString.append(" ");

		converter.str("");

		converter << obj.SIZE[2];

		localString.append(converter.str());
		localString.append("</size>\n");
		localString.append("</box>\n");

	}

	if (obj.SHAPE == "cylinder") {
		localString.append("<radius>");

		converter << obj.SIZE[0];

		localString.append(converter.str());
		localString.append("</radius>");
		localString.append("<length>");

		converter.str("");

		converter << obj.SIZE[1];

		localString.append(converter.str());
		localString.append("</length>");

		localString.append("</cylinder>\n");
	}

	if (obj.SHAPE == "sphere") {

		localString.append("<radius>");

		converter << obj.SIZE[0];

		localString.append(converter.str());
		localString.append("</radius>");

		localString.append("</sphere>\n");

	}

	return localString;

}

std::string RosGetCoord::CreateMaterialColor(gazebo_pkg::Object obj) {

	std::string localString;
	char c;

	localString.append("<material>\n");
	localString.append("<script>\n");
	localString.append(
			"<uri>file://media/materials/scripts/gazebo.material</uri>\n");
	localString.append("<name>Gazebo/");

	c = obj.COLOR[0];
	if (islower(c)) {
		c = toupper(c);
		obj.COLOR[0] = c;
	}

	localString.append(obj.COLOR);
	localString.append("</name>\n");
	localString.append("</script>\n");
	localString.append("</material>\n");

	return localString;

//<material name="orange"/>

}

void RosGetCoord::CreateShape(gazebo_pkg::Object obj) {

	sdf::SDF shapeSDF;

	std::string TextSdf;

	TextSdf.append("<sdf version ='1.4'>\n<model name =");
	TextSdf.append(RosGetCoord::CreateModelNameString(obj));
	TextSdf.append(RosGetCoord::CreatePositionString(obj));
	TextSdf.append(RosGetCoord::CreateOrientationString(obj));

	TextSdf.append(
			"<link name ='link'>\n<collision name ='collision'>\n<geometry>");

	TextSdf.append(RosGetCoord::CreateGeometry(obj));

	TextSdf.append(
			"</geometry>\n</collision>\n<visual name ='visual'>\n<geometry>");

	TextSdf.append(RosGetCoord::CreateGeometry(obj));

	TextSdf.append("</geometry>\n");
	TextSdf.append(RosGetCoord::CreateMaterialColor(obj));

	TextSdf.append("</visual>\n</link>\n</model>\n</sdf>");

	std::cout << TextSdf << std::endl;

	shapeSDF.SetFromString(TextSdf);
	this->parent->InsertModelSDF(shapeSDF);

}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(RosGetCoord)
