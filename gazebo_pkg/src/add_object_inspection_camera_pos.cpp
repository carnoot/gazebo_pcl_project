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
float linear_step_size;

float start_x;
float start_y;

float stop_x;
float stop_y;

float first_turn_x;
float first_turn_y;

float second_turn_x;
float second_turn_y;

bool can_send_next_cam_pos;
bool first_phase;
bool second_phase;
bool third_phase;

void InitCircularParameters() {

	cam_x = 0;
	cam_y = 0;
	cam_z = 0;

	radius = 0.6;
	teta = 0; //moves
	phi = 1.0; //fix

	rotation_step_size = 1;
	can_send_next_cam_pos = true;

}

void InitLinearParameters() {

	first_phase = true;
	second_phase = false;
	third_phase = false;

	start_x = -1.65;
	start_y = -6.9;

	stop_x = -3.2;
	stop_y = -6.9;

	first_turn_x = -1.65;
	first_turn_y = -4.4;

	second_turn_x = -3.2;
	second_turn_y = -4.4;

	cam_x = -1.65;
	cam_y = -6.9;
	cam_z = 1;

	linear_step_size = 0.6;

	can_send_next_cam_pos = true;

}

void NextCircularCameraPosition() {

	teta = teta + rotation_step_size;

	cam_x = radius * cos(teta) * sin(phi) + obj_x;
	cam_y = radius * sin(teta) * sin(phi) + obj_y;
	cam_z = radius * cos(phi) + obj_z;

}

void NextLinearCameraPosition() {

	if (first_phase)
		if (cam_y < first_turn_y) {
			cam_y = cam_y + linear_step_size;
		} else {
			first_phase = false;
			second_phase = true;
		}
	if (second_phase)
		if (cam_x > second_turn_x) {
			cam_x = cam_x - linear_step_size;
		} else {
			second_phase = false;
			third_phase = true;
		}
	if (third_phase)
		if (cam_y > stop_y) {
			cam_y = cam_y - linear_step_size;
		} else {
			third_phase = false;
			first_phase = true;
		}

}

bool CameraCircularMovementStop() {

	bool to_stop = false;

	if (teta > 6.28) {
		to_stop = true;
	} else {
		to_stop = false;
	}

	return (to_stop);

}

bool CameraLinearMovementStop() {

	bool to_stop = false;

	if (first_phase && cam_y < stop_y + linear_step_size) {
		to_stop = true;
	} else {
		to_stop = false;
	}

	return (to_stop);

}

bool callback(gazebo_pkg::ObjectCanSendNextCamPos::Request &req,
		gazebo_pkg::ObjectCanSendNextCamPos::Response &res) {

	can_send_next_cam_pos = true;

	return (true);
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

		InitLinearParameters();
		object_number_srv.request.number = atoll(argv[1]);

		while (ros::ok()) {

			if (can_send_next_cam_pos) {

				NextLinearCameraPosition();

				std::cerr << cam_x << " " << cam_y << " " << cam_z << std::endl;

				camera_pos_srv.request.last = CameraLinearMovementStop();

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
