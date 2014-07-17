#include "ros/ros.h"
#include "gazebo_pkg/ObjectInspectionNumber.h"
#include <cstdlib>

int main(int argc, char **argv) {

	if (argc != 2) {
		ROS_ERROR("Needed a parameter for object number!");
		return 0;
	} else {
		ros::init(argc, argv, "add_object_client");

		ros::NodeHandle n;
		ros::ServiceClient client = n.serviceClient<
				gazebo_pkg::ObjectInspectionNumber>("object_to_inspect");

		gazebo_pkg::ObjectInspectionNumber srv;
		std::cerr << atoll(argv[1]) << std::endl;
		srv.request.number = atoll(argv[1]);
		if (client.call(srv)) {
			ROS_INFO("OK!");
		} else {
			ROS_ERROR("Not OK!");
			return 1;
		}

		return 0;
	}
}
