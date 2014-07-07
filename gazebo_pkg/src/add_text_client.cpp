#include "ros/ros.h"
#include "gazebo_pkg/GetTextParam.h"
#include <cstdlib>

int main(int argc, char **argv) {

	ROS_INFO("ADD TEXT CLIENT STARTED!");

	ros::init(argc, argv, "add_text_client");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_pkg::GetTextParam>(
			"get_text_param");

	std::cout << client.getService() << std::endl;

	gazebo_pkg::GetTextParam srv;


	srv.request.object = "cylinder::link::visual";
		srv.request.customText = "SPHERE";
		srv.request.customFontSize = 1;
		srv.request.customFont = "Arial";
		srv.request.customColor[0] = 0;
		srv.request.customColor[1] = 1;
		srv.request.customColor[2] = 0;
		srv.request.customColor[3] = 1;

//	srv.request.object = "cylinder::link::boxVisual";
//	srv.request.customText = "SPHERE";
//	srv.request.customFontSize = 1;
//	srv.request.customFont = "Arial";
//	srv.request.customColor[0] = 0;
//	srv.request.customColor[1] = 1;
//	srv.request.customColor[2] = 0;
//	srv.request.customColor[3] = 1;

//	srv.request.object = "bowl::bowl_link::bowl_visual";
//		srv.request.customText = "SPHERE";
//		srv.request.customFontSize = 1;
//		srv.request.customFont = "Arial";
//		srv.request.customColor[0] = 0;
//		srv.request.customColor[1] = 1;
//		srv.request.customColor[2] = 0;
//		srv.request.customColor[3] = 1;

	std::cout << "DONE" << std::endl;

	if (client.call(srv)) {
		ROS_INFO("OK!");
	} else {
		ROS_ERROR("Not OK!");
		return 1;
	}

	return 0;
}
