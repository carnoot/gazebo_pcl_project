#include "ros/ros.h"
#include "gazebo_pkg/GetObject.h"
#include "gazebo_pkg/Object.h"
#include <cstdlib>

int main(int argc, char **argv) {
	ros::init(argc, argv, "add_object_client");
//  if (argc != 1)
//  {
//    ROS_INFO("usage: add_object_client ID(int) COLOR(string) SHAPE(string) Pose(Point-x,y,z - Quaternion - i,j,k,w)");
//    return 1;
//  }

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

	sphereSizeVect.push_back(0.2);

	obj.COLOR = "green";
	obj.ID = 1;
	obj.SHAPE = "box";
	obj.SIZE = boxSizeVect;
	obj.pose.position.x = -2.67;
	obj.pose.position.y = -6.06;
	obj.pose.position.z = 3;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "blue";
	obj.ID = 2;
	obj.SHAPE = "cylinder";
	obj.SIZE = cylinderSizeVect;
	obj.pose.position.x = -2.67;
	obj.pose.position.y = -6.8;
	obj.pose.position.z = 3;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	obj.COLOR = "red";
	obj.ID = 3;
	obj.SHAPE = "sphere";
	obj.SIZE = sphereSizeVect;
	obj.pose.position.x = -2.67;
	obj.pose.position.y = -5.3;
	obj.pose.position.z = 3;

	obj.pose.orientation.w = 1;
	obj.pose.orientation.x = 0;
	obj.pose.orientation.y = 0;
	obj.pose.orientation.z = 0;

	test_vect.push_back(obj);

	gazebo_pkg::GetObject srv;
	srv.request.msg = test_vect;
	if (client.call(srv)) {
		ROS_INFO("OK!");
	} else {
		ROS_ERROR("Not OK!");
		return 1;
	}

	return 0;
}
