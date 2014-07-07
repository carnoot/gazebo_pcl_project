#include <iostream>
#include "/opt/ros/hydro/include/ros/ros.h"
#include "gazebo_pkg/GetObject.h"
#include "gazebo_pkg/Object.h"
#include "gazebo_pkg/SendStampedPose.h"
#include <math.h>
#include <cstdlib>
#include "/opt/ros/hydro/include/geometry_msgs/PoseStamped.h"

//#include <eigen3/Eigen/src/Core/Matrix.h>
//#include <eigen3/Eigen/src/Geometry/Quaternion.h>
//#include <eigen3/Eigen/src/Geometry/RotationBase.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "myRosNode");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_pkg::SendStampedPose>("send_stamped_pose");

	gazebo_pkg::SendStampedPose srv;
	srv.request.msg.pose.position.x
	srv.request.msg.pose.position.y
	srv.request.msg.pose.position.z

	srv.request.msg.pose.orientation.w =
	srv.request.msg.pose.orientation.x =
	srv.request.msg.pose.orientation.y =
	srv.request.msg.pose.orientation.z =

//	Eigen::RotationBase rot;
//	Eigen::Quaternionf quat_f;
	Eigen::Matrix3f matr;
//
//	matr(0,0) = 1;
//	float a = matr(0,0);
//	std::cerr << a << std::endl;

//	std::vector<gazebo_pkg::Object> test_vect;


//	StampedPose.pose.position.x = ;
//	StampedPose.pose.position.y = ;
//	StampedPose.pose.position.z = ;

//	srv.request.msg = test_vect;
//	if (client.call(srv)) {
//		ROS_INFO("OK!");
//	} else {
//		ROS_ERROR("Not OK!");
//		return 1;
//	}

	return 0;
}
