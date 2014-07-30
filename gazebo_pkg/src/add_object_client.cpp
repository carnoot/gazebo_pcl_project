#include "ros/ros.h"
#include "gazebo_pkg/GetObject.h"
#include "gazebo_pkg/Object.h"
#include <cstdlib>

float z_coordinate = 1.01;

int main(int argc, char **argv) {
	ros::init(argc, argv, "add_object_client");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_pkg::GetObject>(
			"get_object");

	std::vector<gazebo_pkg::Object> test_vect;

	gazebo_pkg::Object obj;
	std::vector<float> boxSizeVect;
	std::vector<float> cylinderSizeVect;
	std::vector<float> sphereSizeVect;

	boxSizeVect.push_back(0.1);
	boxSizeVect.push_back(0.1);
	boxSizeVect.push_back(0.1);

	cylinderSizeVect.push_back(0.1);
	cylinderSizeVect.push_back(0.1);

	sphereSizeVect.push_back(0.1);

	obj.COLOR = "white";
	obj.ID = 1;
	obj.SHAPE = "box";
	obj.MESH = false;
	obj.SIZE = boxSizeVect;
	obj.CLASSIFIER = 2;
	obj.pose.position.x = -2.42;
	obj.pose.position.y = -5.99;
	obj.pose.position.z = 0.92;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "blue";
	obj.ID = 2;
	obj.SHAPE = "cylinder";
	obj.MESH = false;
	obj.SIZE = cylinderSizeVect;
	obj.CLASSIFIER = 4;
	obj.pose.position.x = -2.67;
	obj.pose.position.y = -6.8;
	obj.pose.position.z = 0.91;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "red";
	obj.ID = 3;
	obj.SHAPE = "pot1";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 4;
	obj.pose.position.x = -2.61;
	obj.pose.position.y = -5.35;
	obj.pose.position.z = 0.959;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "blue";
	obj.ID = 4;
	obj.SHAPE = "bowl1";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 1;
	obj.pose.position.x = -2.1;
	obj.pose.position.y = -5.3;
	obj.pose.position.z = 0.91;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "yellow";
	obj.ID = 5;
	obj.SHAPE = "plate";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 5;
	obj.pose.position.x = -2.37;
	obj.pose.position.y = -5.8;
	obj.pose.position.z = 0.88;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "purple";
	obj.ID = 6;
	obj.SHAPE = "fork";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 6;
	obj.pose.position.x = -2.07;
	obj.pose.position.y = -6.2;
	obj.pose.position.z = 0.87;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "red";
	obj.ID = 7;
	obj.SHAPE = "icedtea";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 2;
	obj.pose.position.x = -2.38;
	obj.pose.position.y = -6.32;
	obj.pose.position.z = 0.865;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "orange";
	obj.ID = 8;
	obj.SHAPE = "teapot";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 1;
	obj.pose.position.x = -2.20;
	obj.pose.position.y = -6.6;
	obj.pose.position.z = 0.87;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "green";
	obj.ID = 9;
	obj.SHAPE = "pancake_maker";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 5;
	obj.pose.position.x = -2.18;
	obj.pose.position.y = -4.87;
	obj.pose.position.z = 0.87;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "yellow";
	obj.ID = 10;
	obj.SHAPE = "plate";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 5;
	obj.pose.position.x = -2.16;
	obj.pose.position.y = -5.55;
	obj.pose.position.z = 0.88;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "purple";
	obj.ID = 11;
	obj.SHAPE = "sphere";
	obj.MESH = false;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 1;
	obj.pose.position.x = -2.12;
	obj.pose.position.y = -5.90;
	obj.pose.position.z = 0.98;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "yellow";
	obj.ID = 12;
	obj.SHAPE = "icedtea_rotated";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 2;
	obj.pose.position.x = -2.19;
	obj.pose.position.y = -6.6;
	obj.pose.position.z = 0.93;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	sphereSizeVect[0] = 0.06;

	obj.COLOR = "green";
	obj.ID = 13;
	obj.SHAPE = "sphere";
	obj.MESH = false;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 1;
	obj.pose.position.x = -2.65;
	obj.pose.position.y = -6.5;
	obj.pose.position.z = 0.92;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "purple";
	obj.ID = 14;
	obj.SHAPE = "bowl1";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 1;
	obj.pose.position.x = -2.56;
	obj.pose.position.y = -6.32;
	obj.pose.position.z = 0.90;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "red";
	obj.ID = 15;
	obj.SHAPE = "mondamin";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 6;
	obj.pose.position.x = -2.56;
	obj.pose.position.y = -6.12;
	obj.pose.position.z = 0.99;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "white";
	obj.ID = 16;
	obj.SHAPE = "pot";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 4;
	obj.pose.position.x = -2.33;
	obj.pose.position.y = -7.05;
	obj.pose.position.z = 0.87;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "orange";
	obj.ID = 17;
	obj.SHAPE = "mug1";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 4;
	obj.pose.position.x = -2.69;
	obj.pose.position.y = -6.2;
	obj.pose.position.z = 0.92;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "blue";
	obj.ID = 18;
	obj.SHAPE = "mondamin";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 6;
	obj.pose.position.x = -2.37;
	obj.pose.position.y = -6.44;
	obj.pose.position.z = 0.98;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "blue";
	obj.ID = 19;
	obj.SHAPE = "mondamin";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 6;
	obj.pose.position.x = -2.59;
	obj.pose.position.y = -5.71;
	obj.pose.position.z = 0.98;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "orange";
	obj.ID = 20;
	obj.SHAPE = "icedtea_rotated";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 2;
	obj.pose.position.x = -2.39;
	obj.pose.position.y = -5.41;
	obj.pose.position.z = 0.93;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "green";
	obj.ID = 21;
	obj.SHAPE = "knife";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 6;
	obj.pose.position.x = -2.06;
	obj.pose.position.y = -6.32;
	obj.pose.position.z = 0.87;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "purple";
	obj.ID = 22;
	obj.SHAPE = "sphere";
	obj.MESH = false;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 1;
	obj.pose.position.x = -2.62;
	obj.pose.position.y = -5.00;
	obj.pose.position.z = 0.95;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "green";
	obj.ID = 23;
	obj.SHAPE = "cylinder";
	obj.MESH = false;
	obj.SIZE = cylinderSizeVect;
	obj.CLASSIFIER = 4;
	obj.pose.position.x = -2.67;
	obj.pose.position.y = -5.94;
	obj.pose.position.z = 0.92;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "orange";
	obj.ID = 24;
	obj.SHAPE = "mondamin";
	obj.MESH = true;
	obj.SIZE = sphereSizeVect;
	obj.CLASSIFIER = 6;
	obj.pose.position.x = -2.33;
	obj.pose.position.y = -5.36;
	obj.pose.position.z = 0.98;
	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	gazebo_pkg::GetObject srv;
	srv.request.msg = test_vect;
	if (client.call(srv)) {
		ROS_INFO("Objects Added Successfully!");
	} else {
		ROS_ERROR("Objects NOT Added!");
		return 1;
	}

	return 0;
}
