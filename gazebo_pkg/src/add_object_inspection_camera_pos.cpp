#include "ros/ros.h"
#include "gazebo_pkg/ObjectInspectionCameraPos.h"
#include "gazebo_pkg/ObjectInspectionNumber.h"
#include "gazebo_pkg/ObjectInspectionStart.h"
#include "gazebo_pkg/ObjectCanSendNextCamPos.h"
#include <gazebo.hh>
#include "gazebo/common/common.hh"
#include <cstdlib>

float radius;
float teta;
float phi;

float cam_x;
float cam_y;
float cam_z;

float obj_x;
float obj_y;
float obj_z;

float rotation_step_size;
bool can_send_next_cam_pos;

void InitParameters() {

	cam_x = 0;
	cam_y = 0;
	cam_z = 0;

	radius = 1;
	teta = 0; //moves
	phi = 1.0; //fix

	rotation_step_size = 1;
	can_send_next_cam_pos = true;

}

void NextCameraPosition() {

	teta = teta + rotation_step_size;

	cam_x = radius * cos(teta) * sin(phi) + obj_x;
	cam_y = radius * sin(teta) * sin(phi) + obj_y;
	cam_z = radius * cos(phi) + obj_z;

}

bool callback(gazebo_pkg::ObjectCanSendNextCamPos::Request &req,
		gazebo_pkg::ObjectCanSendNextCamPos::Response &res) {

	can_send_next_cam_pos = true;

	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "add_object_client");

	if (argc != 2) {
		ROS_ERROR("You need to introduce Object Number!");
		return 0;
	}

	else {
		ros::NodeHandle n;
		ros::ServiceClient camera_pos_client = n.serviceClient<
				gazebo_pkg::ObjectInspectionCameraPos>("get_camera_position");
		ros::ServiceClient object_number_client = n.serviceClient<
				gazebo_pkg::ObjectInspectionNumber>("object_to_inspect");
		ros::ServiceClient object_center_client = n.serviceClient<
				gazebo_pkg::ObjectInspectionStart>("pass_object_center");
		ros::ServiceServer get_can_send_next_cam_pos = n.advertiseService(
				"get_can_send_next_cam_pos", callback);

		gazebo_pkg::ObjectInspectionCameraPos camera_pos_srv;
		gazebo_pkg::ObjectInspectionNumber object_number_srv;
		gazebo_pkg::ObjectInspectionStart object_inspection_start_srv;

		object_inspection_start_srv.request.number = atoll(argv[1]);
		if (object_center_client.call(object_inspection_start_srv)) {
			obj_x = object_inspection_start_srv.response.centerPoint.elems[0];
			obj_y = object_inspection_start_srv.response.centerPoint.elems[1];
			obj_z = object_inspection_start_srv.response.centerPoint.elems[2];
			ROS_INFO("OK!");
		} else {
			ROS_ERROR("Not OK!");
			return 1;
		}

		ros::AsyncSpinner *spinner = new ros::AsyncSpinner(1);
		spinner->start();

		InitParameters();
		object_number_srv.request.number = atoll(argv[1]);

		while (ros::ok()) {

			if (can_send_next_cam_pos) {

				NextCameraPosition();

				std::cerr << cam_x << " " << cam_y << " " << cam_z << std::endl;

				if (teta > 6.28){
					camera_pos_srv.request.last = true;
				}
				else
				{
					camera_pos_srv.request.last = false;
				}

				camera_pos_srv.request.cameraPos.elems[0] = cam_x;
				camera_pos_srv.request.cameraPos.elems[1] = cam_y;
				camera_pos_srv.request.cameraPos.elems[2] = cam_z;

				if (camera_pos_client.call(camera_pos_srv)) {
					ROS_INFO("camera_pos_srv OK!");
				} else {
					ROS_ERROR("camera_pos_srv Not OK!");
					return 1;
				}

				sleep(3);

				if (object_number_client.call(object_number_srv)) {
					ROS_INFO("object_number_client OK!");
				} else {
					ROS_ERROR("object_number_client Not OK!");
					return 1;
				}

				can_send_next_cam_pos = false;

			}

		}
	}

	return 0;
}
