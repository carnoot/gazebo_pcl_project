#include "ros/ros.h"
#include "gazebo_pkg/GetTextParam.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<gazebo_pkg::GetTextParam::Request>(
			"publisher", 1000);

	gazebo_pkg::GetTextParam::Request srv;
	std::vector<gazebo_pkg::GetTextParam::Request> srv_vect;

	srv.object = "bowl::bowl_link::bowl_visual";
	srv.customText = "Bowl";
	srv.customFontSize = 1;
	srv.customFont = "Arial";
	srv.customColor[0] = 0;
	srv.customColor[1] = 1;
	srv.customColor[2] = 0;
	srv.customColor[3] = 1;

	srv_vect.push_back(srv);

	srv.object = "bottle::bottle_link::bottle_bottom_visual";
	srv.customText = "Bottle";
	srv.customFontSize = 1;
	srv.customFont = "Arial";
	srv.customColor[0] = 0;
	srv.customColor[1] = 1;
	srv.customColor[2] = 0;
	srv.customColor[3] = 1;

	srv_vect.push_back(srv);

	srv.object = "bowl::bowl_link::bowl_visual";
	srv.customText = "Bowl";
	srv.customFontSize = 1;
	srv.customFont = "Arial";
	srv.customColor[0] = 0;
	srv.customColor[1] = 1;
	srv.customColor[2] = 0;
	srv.customColor[3] = 1;

	srv_vect.push_back(srv);
//
//	srv.object = "mug::mug_link::mug_visual";
//	srv.customText = "Mug";
//	srv.customFontSize = 1;
//	srv.customFont = "Arial";
//	srv.customColor[0] = 0;
//	srv.customColor[1] = 1;
//	srv.customColor[2] = 0;
//	srv.customColor[3] = 1;
//
//	srv_vect.push_back(srv);

	int i = 0;

	while (ros::ok()) {
		if (i <= srv_vect.size() - 1) {
			std::cout << "Size:" << srv_vect.size() << std::endl;
			std::cout << "publishing:" << srv_vect[i].object << std::endl;
			chatter_pub.publish(srv_vect[i]);
			ros::spinOnce();
			sleep(7);
		}
		i++;
	}

	return 0;
}
