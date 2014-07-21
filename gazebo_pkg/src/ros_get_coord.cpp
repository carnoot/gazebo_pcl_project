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

	this->service_1 = n.advertiseService("object_to_inspect",
			&RosGetCoord::ObjectToInspect, this);

	this->service_2 = n.advertiseService("pass_camera_position",
			&RosGetCoord::PassCameraPosition, this);

	this->service_3 = n.advertiseService("pass_object_center",
			&RosGetCoord::PassObjectCenter, this);

	this->process_offset = 10;

	ROS_INFO("Waiting for coordinates!");

}

void RosGetCoord::OnUpdate() {

//	std::cerr << "Model count: " << this->parent->GetModelCount() << std::endl;
//	for (int i = 0; i < this->parent->GetModelCount(); i++){
//	std::cerr << this->parent->GetModel(i)->GetName() << std::endl;
//	}

}

bool RosGetCoord::PassObjectCenter(
		gazebo_pkg::ObjectInspectionStart::Request &req,
		gazebo_pkg::ObjectInspectionStart::Response &res) {

	int nr = req.number;

	boost::shared_ptr<gazebo::physics::Model> model_ptr =
			this->parent->GetModel(nr);
	physics::ModelPtr my_model = this->parent->GetModel(nr);

	this->bounding_box = math::Box(model_ptr->GetBoundingBox().min,
			model_ptr->GetBoundingBox().max);
	this->object_center.resize(3);

	res.centerPoint.elems[0] = this->bounding_box.GetCenter().x;
	res.centerPoint.elems[1] = this->bounding_box.GetCenter().y;
	res.centerPoint.elems[2] = this->bounding_box.GetCenter().z;

	return true;

}

bool RosGetCoord::PassCameraPosition(
		gazebo_pkg::ObjectInspectionCameraPos::Request &req,
		gazebo_pkg::ObjectInspectionCameraPos::Response &res) {

	this->camera_pos_x = req.cameraPos.elems[0];
	this->camera_pos_y = req.cameraPos.elems[1];
	this->camera_pos_z = req.cameraPos.elems[2];

	std::cout << "Camera POS: " << this->camera_pos_x << " "
			<< this->camera_pos_y << " " << this->camera_pos_z << std::endl;

	std::cerr << "Camera positions passed!" << std::endl;

	return true;

}

bool RosGetCoord::ObjectToInspect(
		gazebo_pkg::ObjectInspectionNumber::Request &req,
		gazebo_pkg::ObjectInspectionNumber::Response &res) {

	int nr = req.number;

	boost::shared_ptr<gazebo::physics::Model> model_ptr =
			this->parent->GetModel(nr);
	physics::ModelPtr my_model = this->parent->GetModel(nr);

	this->bounding_box = math::Box(model_ptr->GetBoundingBox().min,
			model_ptr->GetBoundingBox().max);
	this->object_center.resize(3);

	this->object_center[0] = this->bounding_box.GetCenter().x;
	this->object_center[1] = this->bounding_box.GetCenter().y;
	this->object_center[2] = this->bounding_box.GetCenter().z;

	res.centerPoint.elems[0] = this->bounding_box.GetCenter().x;
	res.centerPoint.elems[1] = this->bounding_box.GetCenter().y;
	res.centerPoint.elems[2] = this->bounding_box.GetCenter().z;

	std::cout << "GOT BOUNDING BOX && CENTER" << std::endl;

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<
			gazebo_pkg::ObjectInspectionCenter>("get_object_center");

	gazebo_pkg::ObjectInspectionCenter srv;
	srv.request.centerPoint.elems[0] = this->object_center[0];
	srv.request.centerPoint.elems[1] = this->object_center[1];
	srv.request.centerPoint.elems[2] = this->object_center[2];

	std::cerr << "Center: " << srv.request.centerPoint.elems[0] << " "
			<< srv.request.centerPoint.elems[1] << " "
			<< srv.request.centerPoint.elems[2] << std::endl;

	if (client.call(srv)) {
		ROS_INFO("Object Center Sent Successfully!");
	} else {
		ROS_ERROR("Object Center Sending Failed!");
	}

	std::cerr << "REAL BOUNDING MIN: " << this->bounding_box.min.x << " "
			<< this->bounding_box.min.y << " " << this->bounding_box.min.z
			<< std::endl;
	std::cerr << "REAL BOUNDING MAX: " << this->bounding_box.max.x << " "
			<< this->bounding_box.max.y << " " << this->bounding_box.max.z
			<< std::endl;

	client = n.serviceClient<gazebo_pkg::ObjectInspectionBounding>(
			"get_object_bounding");

	gazebo_pkg::ObjectInspectionBounding srv1;
//	srv1.request.BoundingMin.elems[0] = this->camera_pos_x - this->bounding_box.min.x;
//	srv1.request.BoundingMin.elems[1] = this->camera_pos_y - this->bounding_box.min.y;
//	srv1.request.BoundingMin.elems[2] = this->camera_pos_z - this->bounding_box.min.z;
//
//	srv1.request.BoundingMax.elems[0] = this->camera_pos_x - this->bounding_box.max.x;
//	srv1.request.BoundingMax.elems[1] = this->camera_pos_y - this->bounding_box.max.y;
//	srv1.request.BoundingMax.elems[2] = this->camera_pos_z - this->bounding_box.max.z;

//################# HIBAS !!! ####################
	srv1.request.BoundingMin.elems[0] = this->bounding_box.min.x
			- this->camera_pos_x; //this->process_offset;
	srv1.request.BoundingMin.elems[1] = this->bounding_box.min.y
			- this->camera_pos_y; //this->process_offset;
	srv1.request.BoundingMin.elems[2] = this->bounding_box.min.z
			- this->camera_pos_z; //this->process_offset;

	srv1.request.BoundingMax.elems[0] = this->bounding_box.max.x
			- this->camera_pos_x; //this->process_offset;
	srv1.request.BoundingMax.elems[1] = this->bounding_box.max.y
			- this->camera_pos_y; //this->process_offset;
	srv1.request.BoundingMax.elems[2] = this->bounding_box.max.z
			- this->camera_pos_z; //+ this->process_offset;
	//################# HIBAS !!! ####################

	std::cerr << "MIN BOUNDING SRV BEFORE: "
			<< srv1.request.BoundingMin.elems[0] << " "
			<< srv1.request.BoundingMin.elems[1] << " "
			<< srv1.request.BoundingMin.elems[2] << std::endl;

	std::cerr << "MAX BOUNDING SRV AFTER: " << srv1.request.BoundingMax.elems[0]
			<< " " << srv1.request.BoundingMax.elems[1] << " "
			<< srv1.request.BoundingMax.elems[2] << std::endl;

	float aux_bounding;

	//#################### HACK ##########################//
	aux_bounding = -srv1.request.BoundingMin.elems[1];
	srv1.request.BoundingMin.elems[1] = -srv1.request.BoundingMax.elems[1];
	srv1.request.BoundingMax.elems[1] = aux_bounding;

	aux_bounding = -srv1.request.BoundingMin.elems[2];
	srv1.request.BoundingMin.elems[2] = -srv1.request.BoundingMax.elems[2];
	srv1.request.BoundingMax.elems[2] = aux_bounding;

	std::cerr << "MIN BOUNDING SRV: " << srv1.request.BoundingMin.elems[0]
			<< " " << srv1.request.BoundingMin.elems[1] << " "
			<< srv1.request.BoundingMin.elems[2] << std::endl;

	std::cerr << "MAX BOUNDING SRV: " << srv1.request.BoundingMax.elems[0]
			<< " " << srv1.request.BoundingMax.elems[1] << " "
			<< srv1.request.BoundingMax.elems[2] << std::endl;
	//#################### HACK ##########################//

//	std::cerr << std::endl;
//
//	std::cerr << "MIN: " << this->bounding_box.min.x << " "
//			<< this->bounding_box.min.y << " " << this->bounding_box.min.z
//			<< std::endl;
//	std::cerr << "MAX: " << this->bounding_box.max.x << " "
//			<< this->bounding_box.max.y << " " << this->bounding_box.max.z
//			<< std::endl;

	if (client.call(srv1)) {
		ROS_INFO("Object BoundingValues Sent Successfully!");
	} else {
		ROS_ERROR("Object BoundingValues Sending Failed!");
	}

	return true;

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

std::string RosGetCoord::CreateMesh(gazebo_pkg::Object obj) {
	std::string localString;

	localString.append("<mesh>\n");
	localString.append("<uri>/home/furdek/catkin_ws/src/gazebo_pkg/models/");
	localString.append(obj.SHAPE);
	localString.append("/meshes/");
	localString.append(obj.SHAPE);
	localString.append(".stl");
	localString.append("</uri>\n");
	localString.append("<scale>1 1 1</scale>\n");
	localString.append("</mesh>");

	return localString;

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

	if (obj.MESH) {

		TextSdf.append(this->CreateMesh(obj));
		TextSdf.append(
				"</geometry>\n</collision>\n<visual name ='visual'>\n<geometry>");
		TextSdf.append(this->CreateMesh(obj));

	} else {

		TextSdf.append(RosGetCoord::CreateGeometry(obj));

		TextSdf.append(
				"</geometry>\n</collision>\n<visual name ='visual'>\n<geometry>");

		TextSdf.append(RosGetCoord::CreateGeometry(obj));

	}

	TextSdf.append("</geometry>\n");
	TextSdf.append(RosGetCoord::CreateMaterialColor(obj));

	TextSdf.append("</visual>\n</link>\n</model>\n</sdf>");

	std::cout << TextSdf << std::endl;

	shapeSDF.SetFromString(TextSdf);
	this->parent->InsertModelSDF(shapeSDF);

}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(RosGetCoord)
