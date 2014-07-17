#include "ros/ros.h"
#include "gazebo_pkg/ObjectInspectionCameraPos.h"
#include <cstdlib>

int main(int argc, char **argv) {
	ros::init(argc, argv, "add_object_client");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_pkg::ObjectInspectionCameraPos>(
			"get_camera_position");

	gazebo_pkg::ObjectInspectionCameraPos srv;
	srv.request.cameraPos.elems[0] = -2.6;
	srv.request.cameraPos.elems[1] = -6.5;
	srv.request.cameraPos.elems[2] = 1.6;

	if (client.call(srv)) {
		ROS_INFO("OK!");
	} else {
		ROS_ERROR("Not OK!");
		return 1;
	}

	return 0;
}
