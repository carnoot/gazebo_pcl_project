#include "MoveKinectCameraPlugin.h"

using namespace gazebo;

MoveKinectCameraPlugin::MoveKinectCameraPlugin() {
	this->initService();
	this->added = true;
	this->test_bool = true;
}

MoveKinectCameraPlugin::~MoveKinectCameraPlugin() {
}

void MoveKinectCameraPlugin::Load(int /*_argc*/, char ** /*_argv*/) {

	this->testMatrix1 = math::Matrix3(1, 1, 1, 2, 1, 1, 3, 1, 1);
	this->testMatrix2 = math::Matrix3(1, 0, 0, 1, 0, 0, 1, 0, 0);
	this->resultMatrix = math::Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0);

	this->x = 0;
	this->y = 0;

	this->r = 1; //2.5
	this->x_offset = 1; // -3.4
	this->y_offset = 1; // -6.0
	this->z_offset = 0.5;

	this->fi = 0.61;
	this->teta = 0; //math::Angle::Pi.Radian() / 4;

	this->stepSize = 0.01;
	this->rotationAngle = 0; //-math::Angle::Pi.Radian() / 7;
	this->rotationStepSize = 0.5;

	this->tolerance = 0.05;

	this->called = false;
	this->present = false;
	this->look_now = false;
	this->position_ready = false;

	this->z = this->z_offset;

	this->cameraLinkName.append("kinect::camera_link::camera_link_visual");
	this->cameraLinkAnimationName.append(
			"kinect::camera_link::camera_link_visual_animation");

	this->updateConnection = event::Events::ConnectPreRender(
			boost::bind(&MoveKinectCameraPlugin::OnUpdate, this));

}

void MoveKinectCameraPlugin::Init() {

	std::cout << "INIT" << std::endl;

	this->sceneManager = gui::get_active_camera()->GetScene()->GetManager();

	this->xRotationMatrix = math::Matrix3(1, 0, 0, 0, cos(this->rotationAngle),
			sin(this->rotationAngle), 0, -sin(this->rotationAngle),
			cos(this->rotationAngle));
	this->yRotationMatrix = math::Matrix3(cos(this->rotationAngle), 0,
			-sin(this->rotationAngle), 0, 1, 0, sin(this->rotationAngle), 0,
			cos(this->rotationAngle));

	std::cout << "before senderNode INIT" << std::endl;

	this->senderNode = transport::NodePtr(new transport::Node());
	this->senderNode->Init("MoveKinectCameraPluginSender");
	transport::run();

	std::cout << "MoveKinectCameraPlugin WAITING for Connection!" << std::endl;

	this->publisher = this->senderNode->Advertise<
			custom_pose_message::msgs::CustomPoseRequest>("customOrientation");

	this->publisher->WaitForConnection();

	std::cout << "MoveKinectCameraPlugin Connection Ready!" << std::endl;

	sleep(5);

	this->receiveNode = transport::NodePtr(new transport::Node());
	this->receiveNode->Init("MoveKinectCameraPluginReceiver");

	this->subscriber = this->receiveNode->Subscribe("customPosition",
			&MoveKinectCameraPlugin::setCameraPosition, this);

	ros::NodeHandle n;
	this->get_object_center_service = n.advertiseService("get_object_center",
			&MoveKinectCameraPlugin::GetObjectCenter, this);

}

bool MoveKinectCameraPlugin::GetObjectCenter(
		gazebo_pkg::ObjectInspectionCenter::Request &req,
		gazebo_pkg::ObjectInspectionCenter::Response &res) {

	this->lookAt_x = req.centerPoint.elems[0];
	this->lookAt_y = req.centerPoint.elems[1];
	this->lookAt_z = req.centerPoint.elems[2];

	std::cout << "OBJECT CENTER SAVED!" << std::endl;

	std::cout << "Object Centre: " << this->lookAt_x << " " << this->lookAt_y
			<< " " << this->lookAt_z << std::endl;

	this->look_now = true; //ONLY USED IF WANT TO use lookAt outside of the callback function!

}

void MoveKinectCameraPlugin::setCameraPosition(
		const boost::shared_ptr<
				const custom_pose_message::msgs::CustomPoseRequest> & pose_msg) {

	this->camera_pos_x = pose_msg->pos_x();
	this->camera_pos_y = pose_msg->pos_y();
	this->camera_pos_z = pose_msg->pos_z();

	this->position_ready = true;

}

void MoveKinectCameraPlugin::NextPoint() {

	std::cout << "CALCULATE NEXT POINT !!!!!!!!!!!!!!!!!" << std::endl;

	this->teta = this->teta + this->rotationStepSize;

	this->x = this->r * sin(this->teta); //+ this->x_offset;
	this->y = this->r * cos(this->teta); //+ this->y_offset;
	this->z = this->z_offset;

//	std::cout << "x^2: " << this->x * this->x << std::endl;
//	std::cout << "y^2: " << this->y * this->y << std::endl;
//	std::cout << "z: " << this->z << std::endl;
//	std::cout << "x^2 + y^2: " << this->x * this->x + this->y * this->y
//			<< std::endl;

//	this->x = this->r * sin(this->fi) * cos(this->teta); //+ x_offset;
//	this->y = this->r * sin(this->fi) * sin(this->teta); //+ y_offset;
//	this->z = this->r * cos(this->teta) + z_offset;

	//this->x = this->r * sin(this->fi) * cos(this->teta); //+ this->x_offset;
	//this->y = this->r * sin(this->fi) * sin(this->teta); //+ this->y_offset;
	//this->z = this->z_offset;
	//this->z = cos(this->teta) + this->z_offset;

	columnVect.x = this->x;
	columnVect.y = this->y;
	columnVect.z = this->z;

	zeroVect.x = 0;
	zeroVect.y = 0;
	zeroVect.z = 0;

	this->pointToRotateMatrix.SetCol(0, columnVect);
	this->pointToRotateMatrix.SetCol(1, zeroVect);
	this->pointToRotateMatrix.SetCol(2, zeroVect);

//	resultMatrix = this->yRotationMatrix.operator *(this->pointToRotateMatrix);

//	this->newPos.pos.x = this->x;
//	this->newPos.pos.y = this->y;
//	this->newPos.pos.z = this->z;

	this->newPos.pos.x = this->x;
	this->newPos.pos.y = this->y;
	this->newPos.pos.z = this->z;

//	this->newPos.pos.x = resultMatrix[0][0];
//	this->newPos.pos.y = resultMatrix[1][0];
//	this->newPos.pos.z = resultMatrix[2][0];

//	this->ogreNewPosVect.x = this->newPos.pos.x - 5;
//	this->ogreNewPosVect.y = this->newPos.pos.y + 5;
//	this->ogreNewPosVect.z = this->newPos.pos.z - 2;

//	this->teta = this->teta + 0.01;

//	if (this->teta > 6.28) {
//		this->teta = 0;
//		this->z = this->z_offset;
//	}
}

bool MoveKinectCameraPlugin::CheckPointReached() {

	if ((this->cameraVisual->GetPose().pos.x
			>= this->newPos.pos.x - this->tolerance
			&& this->cameraVisual->GetPose().pos.x
					<= this->newPos.pos.x + this->tolerance)
			&& (this->cameraVisual->GetPose().pos.y
					>= this->newPos.pos.y - this->tolerance
					&& this->cameraVisual->GetPose().pos.y
							<= this->newPos.pos.y + this->tolerance)
			&& (this->cameraVisual->GetPose().pos.z
					>= this->newPos.pos.z - this->tolerance
					&& this->cameraVisual->GetPose().pos.z
							<= this->newPos.pos.z + this->tolerance)) {
		std::cout << "REACHED" << std::endl;
		return true;
	} else {
		std::cout << "NOT REACHED" << std::endl;
		return false;
	}

}

void MoveKinectCameraPlugin::initService() {

	int argc = 0;
	char ** argv = NULL;

	ros::init(argc, argv, "get_camera_movement");

	ros::NodeHandle n;
	this->move_camera = n.advertiseService("move_camera",
			&MoveKinectCameraPlugin::getTextParam, this);

	this->spinner = new ros::AsyncSpinner(1);
	this->spinner->start();

	ROS_INFO("Waiting for Camera Movement !");
}

bool MoveKinectCameraPlugin::getTextParam(
		gazebo_pkg::GetTextParam::Request &req,
		gazebo_pkg::GetTextParam::Response &res) {

	this->called = true;

}

void MoveKinectCameraPlugin::CalcAndPublishNextPoint() {

	this->NextPoint();

	std::cout << "SceneNode Position:"
			<< this->cameraVisual->GetSceneNode()->getPosition() << std::endl;

	this->cameraVisual->GetSceneNode()->lookAt(*new Ogre::Vector3(0, 0, 0.5),
			Ogre::Node::TS_WORLD, Ogre::Vector3::NEGATIVE_UNIT_Z);

	this->cameraQuaternion.w =
			this->cameraVisual->GetSceneNode()->getOrientation().w;

	this->cameraQuaternion.x =
			this->cameraVisual->GetSceneNode()->getOrientation().x;

	this->cameraQuaternion.y =
			this->cameraVisual->GetSceneNode()->getOrientation().y;

	this->cameraQuaternion.z =
			this->cameraVisual->GetSceneNode()->getOrientation().z;

	std::cout << "SCENE W:" << this->cameraQuaternion.w << "SCENE X:"
			<< this->cameraQuaternion.x << "SCENE Y:"
			<< this->cameraQuaternion.y << "SCENE Z:"
			<< this->cameraQuaternion.z << std::endl;

	this->msgToSend.set_rot_w(this->cameraQuaternion.w);
	this->msgToSend.set_rot_x(this->cameraQuaternion.x);
	this->msgToSend.set_rot_y(this->cameraQuaternion.y);
	this->msgToSend.set_rot_z(this->cameraQuaternion.z);

	std::cout << "NewPose X:" << this->newPos.pos.x << std::endl;
	std::cout << "NewPose Y:" << this->newPos.pos.y << std::endl;
	std::cout << "NewPose Z:" << this->newPos.pos.z << std::endl;

	this->msgToSend.set_pos_x(this->newPos.pos.x);
	this->msgToSend.set_pos_y(this->newPos.pos.y);
	this->msgToSend.set_pos_z(this->newPos.pos.z);

	this->publisher->Publish(this->msgToSend);

}

void MoveKinectCameraPlugin::OnUpdate() {

	if (this->present == false) {
		gui::get_active_camera()->GetScene()->GetVisualsBelowPoint(
				*new math::Vector3(-3.45, -4.35, 4), this->visualVect);
	}

	if (this->visualVect.size() != 0 && this->present == false) {
		std::cout << "Creating AXIS Entity" << std::endl;
		this->myEnt = this->sceneManager->createEntity("axis_shaft");
		this->myEnt->setMaterialName("Gazebo/Red");
		this->myEnt2 = this->sceneManager->createEntity("axis_shaft");
		this->myEnt2->setMaterialName("Gazebo/Green");
		this->myEnt3 = this->sceneManager->createEntity("axis_shaft");
		this->myEnt3->setMaterialName("Gazebo/Blue");
		std::cout << "AXIS Entity CREATED" << std::endl;
		for (int j = 0; j < this->visualVect.size(); j++)
			this->cameraVisual =
					gui::get_active_camera()->GetScene()->GetVisual(
							this->cameraLinkName);
		std::cout << "InsertingMesh" << std::endl;
		this->present = true;
	}

	if (this->called == true) {
		this->CalcAndPublishNextPoint();
		this->called = false;
	}

	if (this->present == true && this->added == true) {

		std::cout << "CREATING CHILD SCENE NODE" << std::endl;
		this->x_axis = gui::get_active_camera()->GetScene()->GetVisual(
				this->cameraLinkName)->GetSceneNode()->createChildSceneNode(
				"x_axis");
		this->x_axis->translate(0.17, 0, 0);
		this->x_axis->yaw(Ogre::Radian(M_PI / 2.0));
		this->x_axis->attachObject((Ogre::MovableObject*) this->myEnt);
		std::cout << "X AXIS CREATED" << std::endl;

		this->y_axis = gui::get_active_camera()->GetScene()->GetVisual(
				this->cameraLinkName)->GetSceneNode()->createChildSceneNode(
				"y_axis");
		this->y_axis->translate(0, 0.17, 0);
		this->y_axis->pitch(Ogre::Radian(M_PI / 2.0));
		this->y_axis->attachObject((Ogre::MovableObject*) this->myEnt2);
		std::cout << "Y AXIS CREATED" << std::endl;

		this->z_axis = gui::get_active_camera()->GetScene()->GetVisual(
				this->cameraLinkName)->GetSceneNode()->createChildSceneNode(
				"z_axis");
		this->z_axis->translate(0, 0, 0.17);
		this->z_axis->attachObject((Ogre::MovableObject*) this->myEnt3);
		std::cout << "Z AXIS CREATED" << std::endl;
		std::cout << "CHILD SCENE NODE CREATED" << std::endl;
		for (int j = 0; j < this->visualVect.size(); j++) {
			std::cout << this->visualVect[j]->GetName() << std::endl;
		}
		this->added = false;
	}

	if (this->present == true && this->added == false) {
		if (this->test_bool) {
			gui::get_active_camera()->GetScene()->GetVisualsBelowPoint(
					*new math::Vector3(-3.984, -4.672, 4), this->visualVect);
			this->test_bool = false;
		}

		if (this->look_now && this->position_ready) {

			std::cout << this->cameraVisual->GetSceneNode()->getOrientation().w
					<< " "
					<< this->cameraVisual->GetSceneNode()->getOrientation().x
					<< " "
					<< this->cameraVisual->GetSceneNode()->getOrientation().y
					<< " "
					<< this->cameraVisual->GetSceneNode()->getOrientation().z
					<< std::endl;

			this->cameraVisual->GetSceneNode()->lookAt(
					*new Ogre::Vector3(this->lookAt_x, this->lookAt_y,
							this->lookAt_z), Ogre::Node::TS_WORLD,
					Ogre::Vector3::UNIT_X);

			this->msgToSend.set_pos_x(
					this->cameraVisual->GetSceneNode()->getPosition().x);
			this->msgToSend.set_pos_y(
					this->cameraVisual->GetSceneNode()->getPosition().y);
			this->msgToSend.set_pos_z(
					this->cameraVisual->GetSceneNode()->getPosition().z);

			this->msgToSend.set_rot_w(
					this->cameraVisual->GetSceneNode()->getOrientation().w);
			this->msgToSend.set_rot_x(
					this->cameraVisual->GetSceneNode()->getOrientation().x);
			this->msgToSend.set_rot_y(
					this->cameraVisual->GetSceneNode()->getOrientation().y);
			this->msgToSend.set_rot_z(
					this->cameraVisual->GetSceneNode()->getOrientation().z);

			std::cerr << this->cameraVisual->GetSceneNode()->getOrientation()
					<< std::endl;

			this->cameraVisual->GetSceneNode()->setOrientation(1,0,0,0);

			this->publisher->Publish(this->msgToSend);

			this->look_now = false;
			this->position_ready = false;

		}
	}
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN (MoveKinectCameraPlugin)
