#include "MoveKinectCamera.h"

using namespace gazebo;

MoveCamera::MoveCamera() {

//	std::cout << "CONSTRUCTOR" << std::endl;

	ROS_INFO("CONSTRUCTOR OF MoveCamera");

	this->i = 0;

	this->rotationTest = 0;
	this->yaw = -80;

	this->tfKinectFrameName = "head_mount_kinect_rgb_optical_frame";
	this->tfFixedFrameName = "/map";
	this->tfServiceName = "/tf";
	this->tfKitchenLinkName = "iai_kitchen/kitchen_link";

	this->kitchen_link_x_offset = -3.45;
	this->kitchen_link_y_offset = -4.35;
	this->kitchen_link_z_offset = 0; //0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	this->kitchen_link_roll_offset = 0;
	this->kitchen_link_pitch_offset = 0;
	this->kitchen_link_yaw_offset = 0; //-1.57; //-3.141;

//	this->initialQuaternion.setW();
//	this->initialQuaternion.setX();
//	this->initialQuaternion.setY();
//	this->initialQuaternion.setZ();

//	std::cout << "ROT 3.14" << std::endl;
//	this->auxQuaternion.SetFromEuler(0, 0, 1.57); // 270 degree rotation around x = -4.71 radians
//	this->auxQuaternion.Normalize();

//	this->auxQuaternion.SetFromEuler(-1.530, -0.006, 1.533); // ROBOT LOOKING AT FERENC'S TABLE
//	this->auxQuaternion.SetFromEuler(0,0,3.14);
//	this->auxQuaternion.Normalize();

}

MoveCamera::~MoveCamera() {
}

void MoveCamera::Load(physics::ModelPtr _parent, sdf::ElementPtr _element) {

	this->i = 0;
	this->j = 0;

	std::cout << "SLEEP" << std::endl;

	sleep(3);

	std::cout << "SUBSCRIBING" << std::endl;

	this->receiveNode = transport::NodePtr(new transport::Node());
	this->receiveNode->Init("MoveKinectCameraReceiver");

	this->subscriber = this->receiveNode->Subscribe("customOrientation",
			&MoveCamera::SetNewOrientation, this);

	this->senderNode = transport::NodePtr(new transport::Node());
	this->senderNode->Init("MoveKinectCameraSender");
	transport::run();

	this->publisher = this->senderNode->Advertise
			< custom_pose_message::msgs::CustomPoseRequest
			> ("customPosition");

	std::cout << "MoveKinectCamera WAITING for Connection!" << std::endl;
	this->publisher->WaitForConnection();
	std::cout << "MoveKinectCamera Connection Ready!" << std::endl;

	this->msgToSend.set_pos_x(10.0);
	this->msgToSend.set_pos_y(20.0);
	this->msgToSend.set_pos_z(30.0);

	this->msgToSend.set_rot_w(1.0);
	this->msgToSend.set_rot_x(0.0);
	this->msgToSend.set_rot_y(0.0);
	this->msgToSend.set_rot_z(0.0);

	this->r = 0.6283; //Pt Stefan 62.83 cm raza pana la obiect

	this->x_offset = 1;
	this->y_offset = 5;
	this->z_offset = 0.4987; // Pt Stefan 0.49 distanta dintre ground si camera

	this->z = this->z_offset;

	this->fi = 0.61;
	this->teta = 0; //math::Angle::Pi.Radian() / 4;

	this->stepSize = 0.01;
	this->rotationAngle = -math::Angle::Pi.Radian() / 4;
	this->cameraRotationStepSize = 0.5;

//	this->xRotationMatrix = math::Matrix3(1, 0, 0, 0, cos(this->rotationAngle),
//			sin(this->rotationAngle), 0, -sin(this->rotationAngle),
//			cos(this->rotationAngle));
//	this->yRotationMatrix = math::Matrix3(cos(this->rotationAngle), 0,
//			-sin(this->rotationAngle), 0, 1, 0, sin(this->rotationAngle), 0,
//			cos(this->rotationAngle));

	this->cameraLinkName.append("camera_link");
	this->model = _parent;

//	std::cout << "MODEL'S NAME: " << this->model->GetName() << std::endl;
//	std::cout << this->model->GetLink(this->cameraLinkName)->GetWorldPose().rot
//			<< std::endl;

//	this->newPos.pos.z = this->z_offset;

//	this->newPos.rot.SetFromEuler(-6.28 * 74.62 / 360, 1.57,
//			this->yaw * 6.28 / 360);
//	this->model->SetWorldPose(this->newPos);

//	this->newPos.rot.SetFromEuler(0, 0, 3.1415);
//	this->model->SetWorldPose(this->newPos);

//	this->newPos.rot.SetFromEuler(0, 0, 3.1415); // ROTATIE IN JURUL AXEI Y
//
//	this->model->SetWorldPose(this->newPos);
////
//	this->newPos.rot.SetFromEuler(0, 0, this->yaw * 6.28 / 360); // ROTATIE IN JURUL AXEI Z
//
//	this->model->SetWorldPose(this->newPos);

//	std::cout << "SETTING Y ROTATION" << std::endl;

//	this->newPos.rot.SetFromEuler(6.28*37.62 / 360, 0 , 0); // ROTATIE IN JURUL AXEI X, unghiul dintre planul OX si planul de view al camerei sa fie 37.. grade
//
//	this->model->SetWorldPose(this->newPos);

	this->yaw = 0;

//	this->model->SetLinkWorldPose(this->newPos, this->cameraLinkName);

//	this->model->SetLinkWorldPose(
//			*new math::Pose(this->x_offset, this->y_offset, this->z_offset, 0,
//					0, 0), this->cameraLinkName);

//	this->subscriber = this->receiveNode->Subscribe("~/customOrientation",
//			MoveCamera::SetNewOrientation);
//	this->subscriber = this->receiveNode->Subscribe("~/customOrientation",
//			&MoveCamera::SetNewOrientation, this, false);

//	this->subscriber = this->receiveNode->Subscribe("~/customOrientation",
//			&MoveCamera::SetNewOrientation, this, false);

//	this->subscriber = this->receiveNode->Subscribe("~/customOrientation",
//			&MoveCamera::SetNewOrientation, this, false);

//	std::cout << "CONNECT TO ON UPDATE" << std::endl;

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&MoveCamera::OnUpdate, this));

}

void MoveCamera::SetNewOrientation(
		const boost::shared_ptr<
				const custom_pose_message::msgs::CustomPoseRequest>& orientation_msg) {

	this->look_at_pos_x = orientation_msg->pos_x();
	this->look_at_pos_y = orientation_msg->pos_y();
	this->look_at_pos_z = orientation_msg->pos_z();

	this->look_at_w = orientation_msg->rot_w();
	this->look_at_x = orientation_msg->rot_x();
	this->look_at_y = orientation_msg->rot_y();
	this->look_at_z = orientation_msg->rot_z();

	std::cout << "RECEIVED W = " << orientation_msg->rot_w() << std::endl;
	std::cout << "RECEIVED X = " << orientation_msg->rot_x() << std::endl;
	std::cout << "RECEIVED Y = " << orientation_msg->rot_y() << std::endl;
	std::cout << "RECEIVED Z = " << orientation_msg->rot_z() << std::endl;

}

void MoveCamera::InitCameraNode() {
//	this->sceneManager = this->model->GetWorld()->sceneMsg

}

void MoveCamera::FetchRealKinectPose() {

	math::Quaternion myQuatt;

	this->newPos.pos.x = 0;
	this->newPos.pos.y = 0;
	this->newPos.pos.z = 0;

	this->newPos.rot.w = 0;
	this->newPos.rot.x = 0;
	this->newPos.rot.y = 0;
	this->newPos.rot.z = 0;

	const tf::Vector3 *myVect = new tf::Vector3(0.534, 0.322, 1.760);
	myQuatt.SetFromEuler(-1.530, -0.006, 1.533);

	this->transform.setOrigin(*myVect);
	this->transform.setRotation(
			*new tf::Quaternion(myQuatt.x, myQuatt.y, myQuatt.z, myQuatt.w));

//	try {
//		this->listener.waitForTransform(this->tfFixedFrameName,
//				this->tfKinectFrameName, ros::Time(0), ros::Duration(0.1));
//		this->listener.lookupTransform(this->tfFixedFrameName,
//				this->tfKinectFrameName, ros::Time(0), this->transform);
//	} catch (tf::TransformException &ex) {
//		ROS_ERROR("%s", ex.what());
//	}

//	this->PrintTransformPose();

}

void MoveCamera::SetTransformQuaternion() {

	this->finalQuaternion.w = this->transform.getRotation().getW();
	this->finalQuaternion.x = this->transform.getRotation().getX();
	this->finalQuaternion.y = this->transform.getRotation().getY();
	this->finalQuaternion.z = this->transform.getRotation().getZ();

}

void MoveCamera::SubtractQuaternionAngles() {

	// ******************* TEST **************** //

//	this->auxQuaternion.SetFromEuler(0, 0, 2.14);
//	this->initialQuaternion.SetFromEuler(0, 0, 3.14);
//	this->auxQuaternion.Invert();
//	this->initialQuaternion.operator *=(this->auxQuaternion);
//	std::cout << this->initialQuaternion.GetRoll()
//			<< this->initialQuaternion.GetPitch()
//			<< this->initialQuaternion.GetYaw() << std::endl;

	// ******************* TEST **************** //

	this->initialQuaternion.w = this->transform.getRotation().getW();
	this->initialQuaternion.x = this->transform.getRotation().getX();
	this->initialQuaternion.y = this->transform.getRotation().getY();
	this->initialQuaternion.z = this->transform.getRotation().getZ();

//	this->initialQuaternion.w = 1;
//	this->initialQuaternion.x = 0;
//	this->initialQuaternion.y = 0;
//	this->initialQuaternion.z = 0;

	this->auxQuaternion.SetFromEuler(0, 0, -1.57);
	this->auxQuaternion.Normalize();

	this->initialQuaternion.Invert();
	this->initialQuaternion.operator *=(this->auxQuaternion);

	this->auxQuaternion.SetFromEuler(-1.57, 0, 0);
	this->auxQuaternion.Normalize();
	this->initialQuaternion.operator *=(this->auxQuaternion);

	this->auxQuaternion.SetFromEuler(0, 0, 3.14);
	this->auxQuaternion.Normalize();
	this->initialQuaternion.operator *=(this->auxQuaternion);
//
	this->initialQuaternion.Invert();
	this->auxQuaternion.SetFromEuler(3.14, 0, 0);
	this->auxQuaternion.Normalize();
	this->initialQuaternion.operator *=(this->auxQuaternion);

//	this->auxQuaternion.SetFromEuler(-1.57, 0, 0);
//	this->auxQuaternion.Normalize();
//	this->initialQuaternion.operator *=(this->auxQuaternion);

//	this->auxQuaternion.SetFromEuler(0, 1.57, 0);
//	this->auxQuaternion.Normalize();camera depth image raw ros
//
//	this->initialQuaternion.Invert();
//	this->initialQuaternion.Normalize();
//	this->initialQuaternion.operator *=(this->auxQuaternion);
//	this->initialQuaternion.Normalize();

//	this->auxQuaternion.SetFromEuler(0, 1.57, 0);
//	this->initialQuaternion.operator *=(this->auxQuaternion);
//	this->auxQuaternion.Normalize();

//	this->auxQuaternion.SetFromEuler(0,0,3.14);
//	this->auxQuaternion.Normalize();
//	this->initialQuaternion.operator *(this->auxQuaternion);

	this->finalQuaternion = this->initialQuaternion;

//	std::cout << this->finalQuaternion.GetAsEuler().x
//			<< this->finalQuaternion.GetAsEuler().y
//			<< this->finalQuaternion.GetAsEuler().z << std::endl;
}

void MoveCamera::SetCameraPosition() {

//	std::cout << "Setting TF Position" << std::endl;

	math::Quaternion *myQuat;

	// ################## IMPORTANT ##################### //
//	this->AdjustPositionToKitchenLink();
	// ################## IMPORTANT ##################### //

//	this->TransformRotationToEuler();
//	this->AdjustRotationToKitchenLink();
//	this->TransformEulerToRotation();

//	this->newPos.rot.w = this->rotationW;
//	this->newPos.rot.x = this->rotationX;
//	this->newPos.rot.y = this->rotationY;
//	this->newPos.rot.z = this->rotationZ;

//	this->newPos.pos.x = this->newPos.pos.x - this->transform.getOrigin().x();
//	this->newPos.pos.y = this->newPos.pos.y - this->transform.getOrigin().y();
//	this->newPos.pos.z = this->newPos.pos.z - this->transform.getOrigin().z();

//	this->newPos.rot.w = 1;
//	this->newPos.rot.x = 0;
//	this->newPos.rot.y = 0;
//	this->newPos.rot.z = 0;

//	std::cout << this->finalQuaternion.GetAsEuler().x << " "
//			<< this->finalQuaternion.GetAsEuler().y << " "
//			<< this->finalQuaternion.GetAsEuler().z << std::endl;

//	std::cout << this->finalQuaternion.w << " " << this->finalQuaternion.x
//			<< " " << this->finalQuaternion.y << " " << this->finalQuaternion.z
//			<< std::endl;

	//-Z -X Y in quaternion

	myQuat = new math::Quaternion(this->finalQuaternion.GetAsEuler().z,
			this->finalQuaternion.GetAsEuler().x,
			this->finalQuaternion.GetAsEuler().y);

//	std::cout << myQuat->w << " " << myQuat->x << " " << myQuat->y << " "
//			<< myQuat->z << std::endl;

	// UN-COMMENT //
//	this->newPos.rot.w = this->finalQuaternion.w;
//	this->newPos.rot.x = -this->finalQuaternion.z;
//	this->newPos.rot.y = -this->finalQuaternion.x;
//	this->newPos.rot.z = this->finalQuaternion.y;
	// UN-COMMENT //

	this->newPos.pos.x = this->look_at_pos_x;
	this->newPos.pos.y = this->look_at_pos_y;
	this->newPos.pos.z = this->look_at_pos_z;

	this->newPos.rot.w = this->look_at_w;
	this->newPos.rot.x = this->look_at_x;
	this->newPos.rot.y = this->look_at_y;
	this->newPos.rot.z = this->look_at_z;

//	this->newPos.rot.w = this->finalQuaternion.w;
//	this->newPos.rot.x = this->finalQuaternion.x;
//	this->newPos.rot.y = this->finalQuaternion.y;
//	this->newPos.rot.z = this->finalQuaternion.z;

	this->model->SetWorldPose(this->newPos);
//	this->model->SetLinkWorldPose(this->probaPos, this->cameraLinkName);

}

void MoveCamera::AdjustPositionToKitchenLink() {

//	std::cout << "Orig X: " << this->transform.getOrigin().x() << "Orig Y: "
//			<< this->transform.getOrigin().y() << "Orig Z: "
//			<< this->transform.getOrigin().z() << std::endl;

//	std::cout << "Offset:" << this->kitchen_link_x_offset << ", "
//			<< this->kitchen_link_y_offset << ", "
//			<< this->kitchen_link_z_offset << std::endl;
//
//	std::cout << "newPos BEFORE adjust:" << this->newPos.pos.x << ", "
//			<< this->newPos.pos.y << ", " << this->newPos.pos.z << std::endl;

//	this->newPos.pos.x = (-3.45 - this->transform.getOrigin().x());
//	this->newPos.pos.y = (-4.35 - this->transform.getOrigin().y());

//	this->newPos.pos.x = 1;
//	this->newPos.pos.y = 1;

	this->newPos.pos.x = (double) (this->kitchen_link_x_offset
			- this->transform.getOrigin().x());
	this->newPos.pos.y = (double) (this->kitchen_link_y_offset
			- this->transform.getOrigin().y());
	this->newPos.pos.z = (double) (this->transform.getOrigin().z()
			+ this->kitchen_link_z_offset);

//	std::cout << "newPos AFTER adjust:" << this->newPos.pos.x << ", "
//			<< this->newPos.pos.y << ", " << this->newPos.pos.z << std::endl;

}

void MoveCamera::PrintCameraPose() {

	std::cout << "CAMERA X: "
			<< this->model->GetLink(this->cameraLinkName)->GetWorldPose().pos.x
			<< " Y: "
			<< this->model->GetLink(this->cameraLinkName)->GetWorldPose().pos.y
			<< " Z: "
			<< this->model->GetLink(this->cameraLinkName)->GetWorldPose().pos.z
			<< std::endl;

	std::cout << "CAMERA ROLL: "
			<< this->model->GetLink(this->cameraLinkName)->GetWorldPose().rot.GetAsEuler().x
			<< " PITCH: "
			<< this->model->GetLink(this->cameraLinkName)->GetWorldPose().rot.GetAsEuler().y
			<< " YAW: "
			<< this->model->GetLink(this->cameraLinkName)->GetWorldPose().rot.GetAsEuler().z
			<< std::endl;

}

void MoveCamera::PrintTransformPose() {

	std::cout << "POSE.pos: " << "X: " << this->transform.getOrigin().getX()
			<< "Y: " << this->transform.getOrigin().getY() << "Z: "
			<< this->transform.getOrigin().getZ() << std::endl;
	std::cout << "POSE.rot: " << "W: " << this->transform.getRotation().getW()
			<< "X: " << this->transform.getRotation().getX() << "Y: "
			<< this->transform.getRotation().getY() << "Z: "
			<< this->transform.getRotation().getZ() << std::endl;

}

void MoveCamera::TransformRotationToEuler() {
	math::Quaternion *myQuat = new math::Quaternion(
			(double) this->transform.getRotation().getW(),
			(double) this->transform.getRotation().getX(),
			(double) this->transform.getRotation().getY(),
			(double) this->transform.getRotation().getZ());

	this->rollAngle = myQuat->GetAsEuler().x;
	this->pitchAngle = myQuat->GetAsEuler().y;
	this->yawAngle = myQuat->GetAsEuler().z;

	std::cout << "Orig ROLL: " << this->rollAngle << "Orig PITCH: "
			<< this->pitchAngle << "Orig YAW: " << this->yawAngle << std::endl;

}

void MoveCamera::TransformEulerToRotation() {
	math::Quaternion *auxQuat = new math::Quaternion(this->rollAngle,
			this->pitchAngle, this->yawAngle);

	this->rotationW = auxQuat->w;
	this->rotationX = auxQuat->x;
	this->rotationY = auxQuat->y;
	this->rotationZ = auxQuat->z;
}

void MoveCamera::AdjustRotationToKitchenLink() {
	this->rollAngle = this->rollAngle + this->kitchen_link_roll_offset;
	this->pitchAngle = this->pitchAngle + this->kitchen_link_pitch_offset;
	this->yawAngle = this->yawAngle + this->kitchen_link_yaw_offset;

}

void MoveCamera::OnUpdate() {

	this->i++;
	this->publisher->Publish(this->msgToSend);
//	std::cout << "OnUpdate" << std::endl;
//	std::cout << i << std::endl;

//	this->PrintCameraPose();

	if (this->i % 1000 == 0) {
//		std::cout << "Executing" << std::endl;
		this->FetchRealKinectPose();
		this->SubtractQuaternionAngles();
//		this->PrintTransformPose(); // doar print ramane comentat
		this->SetCameraPosition();
//		this->PrintCameraPose(); // doar print ramane comentat

//		this->newPos.pos.x = this->newPos.pos.z + 0.01;
//		this->model->SetWorldPose(this->newPos);
	}

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (MoveCamera)
