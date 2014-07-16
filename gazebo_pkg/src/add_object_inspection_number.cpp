#include "ros/ros.h"
#include "gazebo_pkg/ObjectInspectionNumber.h"
#include <cstdlib>

int main(int argc, char **argv) {
	ros::init(argc, argv, "add_object_client");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_pkg::ObjectInspectionNumber>(
			"object_to_inspect");

	gazebo_pkg::ObjectInspectionNumber srv;
	srv.request.number = 3;
	if (client.call(srv)) {
		ROS_INFO("OK!");
	} else {
		ROS_ERROR("Not OK!");
		return 1;
	}

	return 0;
}
