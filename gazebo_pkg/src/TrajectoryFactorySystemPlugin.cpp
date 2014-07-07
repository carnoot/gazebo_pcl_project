#include "TrajectoryFactorySystemPlugin.h"

using namespace gazebo;
using namespace rendering;

#define NR_TRAJ 3

//////////////////////////////////////////////////
TrajectoryFactorySystemPlugin::TrajectoryFactorySystemPlugin()
: pos(Ogre::Vector3(0,0,1))
{

	std::cout << "********** CONSTRUCTOR ********" << std::endl;

	// initialize frame
	this->frame = 0;


	this->positionSpline = NULL;
	this->rotationSpline = NULL;
}

//////////////////////////////////////////////////
TrajectoryFactorySystemPlugin::~TrajectoryFactorySystemPlugin()
{
	delete this->positionSpline;
	delete this->rotationSpline;
}

//////////////////////////////////////////////////
void TrajectoryFactorySystemPlugin::Load(int /*_argc*/, char ** /*_argv*/)
{
	std::cout << "********** Trajectory Factory Started ********" << std::endl;

//	// read trajectories with mug in hand
//	for (int i = 3; i <= NR_TRAJ; i++)
//	{
//		this->mugTrajectories.push_back(Trajectory());
//
//		// set file name
//		std::stringstream ss;
//		ss << "pushoff" << i << "_mug.csv";
//
//		this->mugTrajectories.back().SetPosesFromFile(ss.str());
//
//		// reset file name
//		ss.str("");
//	};
//
//	// read trajectories with spatula in hand
//	for (int i = 3; i <= NR_TRAJ; i++)
//	{
//		this->spatulaTrajectories.push_back(Trajectory());
//
//		// set file name
//		std::stringstream ss;
//		ss << "pushoff" << i << "_spatula.csv";
//
//		this->spatulaTrajectories.back().SetPosesFromFile(ss.str());
//
//		// reset file name
//		ss.str("");
//	};

}

//////////////////////////////////////////////////
void TrajectoryFactorySystemPlugin::Init()
{
	// Get the Scene
	this->mScene = gui::get_active_camera()->GetScene();

	// Get the Scene Manager
	this->mSceneMgr = this->mScene->GetManager();


	std::cout << "1" << std::endl;

	// Thread update
	this->updateThread = new boost::thread(boost::bind(
			&TrajectoryFactorySystemPlugin::DrawTrajectories, this));

	std::cout << "2" << std::endl;

	// Event update
	this->updateConnection = event::Events::ConnectPreRender(
			boost::bind(&TrajectoryFactorySystemPlugin::OnUpdate, this));

	std::cout << "3" << std::endl;

	std::vector<Ogre::Entity*> mEntities;

	mEntities.push_back(this->mSceneMgr->createEntity(
							Ogre::SceneManager::PT_SPHERE));

	std::cout << "4" << std::endl;

					// set material
//					this->mEntities.back()->setMaterialName(_color.c_str());

	// Initialize all the mug trajectories
	for (unsigned int i = 0; i < this->mugTrajectories.size(); i++)
	{
		// set up trajectory entities

		std::cout << "!!!!!!!!!adghsfghfgh!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;



		this->mugTrajectories[i].SetEntities(this->mSceneMgr, "Gazebo/Gray");

		std::cout << "!!!!!!!!!adghsfghfgh!!!!!!!!!!!!!!!!!!!!345356456!!!" << std::endl;
		// set up trajectory scene nodes, not visible
		this->mugTrajectories[i].SetSceneNodes(this->mSceneMgr, 0.00025, false);
		// set line points
		this->mugTrajectories[i].SetLinePoints(this->mSceneMgr,"Gazebo/Yellow", true);
	}
	// Initialize all the spatula trajectories
	for (unsigned int i = 0; i < this->spatulaTrajectories.size(); i++)
	{
		// set up trajectory entities
		this->spatulaTrajectories[i].SetEntities(this->mSceneMgr, "Gazebo/Gray");
		// set up trajectory scene nodes, not visible
		this->spatulaTrajectories[i].SetSceneNodes(this->mSceneMgr, 0.00025, false);
		// set line points
		this->spatulaTrajectories[i].SetLinePoints(this->mSceneMgr,"Gazebo/Blue", true);
	}
}

//////////////////////////////////////////////////
void TrajectoryFactorySystemPlugin::OnUpdate()
{
//	math::Pose pose;
//	this->frame += 0.01; // 0 = first spline frame, 1 = last frame
//
//	// interpolation testing
//	if (this->frame < 1)
//	{
//		// first spline
//		pose = this->GetPoseFromSpline(frame,
//				this->poseSplines[0].first,
//				this->poseSplines[0].second);
//
//		this->entityTrajectories[0].push_back(this->CreateEntity("Gazebo/Yellow"));
//		this->sceneNodeTrajectories[0].push_back(this->CreateSceneNode(
//				*this->entityTrajectories[0].back(),
//				0.001,
//				Ogre::Vector3(pose.pos.x,pose.pos.y,pose.pos.z)));
//
//		// second spline
//		pose = this->GetPoseFromSpline(frame,
//				this->poseSplines[1].first,
//				this->poseSplines[1].second);
//
//		this->entityTrajectories[1].push_back(this->CreateEntity("Gazebo/Red"));
//		this->sceneNodeTrajectories[1].push_back(this->CreateSceneNode(
//				*this->entityTrajectories[1].back(),
//				0.001,
//				Ogre::Vector3(pose.pos.x,pose.pos.y,pose.pos.z)));
//	}
}

//////////////////////////////////////////////////
void TrajectoryFactorySystemPlugin::DrawTrajectories()
{
	// initialize frame
	unsigned int frame = 0;
	// initialize the number of trajectories drawn
	unsigned int finished_drawings = 0;

    while(finished_drawings <
    		(this->mugTrajectories.size() + this->spatulaTrajectories.size()))
    {
    	// wait given miliseconds
        boost::this_thread::sleep(
                boost::posix_time::milliseconds(100));

        // loop through all trajectories with mug in hand
    	for (unsigned int i = 0; i < this->mugTrajectories.size(); i++)
    	{
    		if (this->mugTrajectories[i].finished_drawing == true) break;

    		this->mugTrajectories[i].VisualizeFrame(
    				frame,"Gazebo/Yellow",true,0.0001);

    		if (this->mugTrajectories[i].finished_drawing == true)
    		{
    			finished_drawings++;
    		}
    	}

        // loop through all trajectories with spatula in hand
    	for (unsigned int i = 0; i < this->spatulaTrajectories.size(); i++)
    	{

    		if (this->spatulaTrajectories[i].finished_drawing == true) break;

    		if (frame != 71)
    		{
    			this->spatulaTrajectories[i].VisualizeFrame(
    				frame,"Gazebo/Blue",true,0.0001);
    		}
    		else
    		{
        		this->spatulaTrajectories[i].VisualizeFrame(
        				frame,"Gazebo/Red",true,0.00055);
    		}

    		if (this->spatulaTrajectories[i].finished_drawing == true)
    		{
    			finished_drawings++;
    		}
    	}

    	// increment frame
    	frame++;
    }

    std::cout << "Finished Drawing all trajectories" << std::endl;
}

//////////////////////////////////////////////////
Ogre::Entity* TrajectoryFactorySystemPlugin::CreateEntity(std::string _color)
{
	// init entity
	Ogre::Entity* entity = this->mSceneMgr->createEntity(
			Ogre::SceneManager::PT_SPHERE);

	// set material
	entity->setMaterialName(_color.c_str());

	return entity;
}

//////////////////////////////////////////////////
Ogre::SceneNode* TrajectoryFactorySystemPlugin::CreateSceneNode(
		Ogre::Entity &_entity,
		float _scale,
		Ogre::Vector3 _pos)
{
	// init Scene Node
	Ogre::SceneNode *scene_node = this->mSceneMgr->getRootSceneNode(
			)->createChildSceneNode();

	// attach entity
	scene_node->attachObject(&_entity);

	// set scale
	scene_node->setScale(_scale, _scale, _scale);

	// set init position
	scene_node->setPosition(_pos);

	return scene_node;
}

//////////////////////////////////////////////////
void TrajectoryFactorySystemPlugin::SetTransparency(
		Ogre::Entity* _entity,
		float _transparency)
{
	// get material
	Ogre::MaterialPtr _material = _entity->getSubEntity(0)->getMaterial();

    unsigned int techniqueCount, passCount;
    Ogre::Technique *technique;
    Ogre::Pass *pass;
    Ogre::ColourValue dc;

    for (techniqueCount = 0; techniqueCount < _material->getNumTechniques();
         techniqueCount++)
    {
      technique = _material->getTechnique(techniqueCount);

      for (passCount = 0; passCount < technique->getNumPasses(); passCount++)
      {
        pass = technique->getPass(passCount);
        // Need to fix transparency
        if (!pass->isProgrammable() &&
            pass->getPolygonMode() == Ogre::PM_SOLID)
        {
          pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        }

        if (_transparency > 0.0)
        {
          pass->setDepthWriteEnabled(false);
          pass->setDepthCheckEnabled(true);
        }
        else
        {
          pass->setDepthWriteEnabled(true);
          pass->setDepthCheckEnabled(true);
        }

        dc = pass->getDiffuse();
        dc.a =(1.0f - _transparency);
        pass->setDiffuse(dc);
      }
    }
}

//////////////////////////////////////////////////
void TrajectoryFactorySystemPlugin::CreateArrowEntity(
		std::string _headColor,
		std::string _shaftColor,
		Ogre::Vector3 _pos,
		Ogre::Quaternion _orientation,
		Ogre::Vector3 _scale)
{

	this->arrowVisPtr =  boost::make_shared<rendering::Visual>
	(rendering::Visual("arrow_visual", this->mScene, false));

	this->arrowVisPtr->InsertMesh("axis_shaft");
	this->arrowVisPtr->InsertMesh("axis_head");

	this->arrowShaftEntities.push_back(this->mSceneMgr->createEntity("axis_shaft"));
	this->arrowHeadEntities.push_back(this->mSceneMgr->createEntity("axis_head"));

	Ogre::MovableObject* shaftObj = (Ogre::MovableObject*) this->arrowShaftEntities.back();
	Ogre::MovableObject* headObj = (Ogre::MovableObject*) this->arrowHeadEntities.back();

	// set the material of the entities
    if (dynamic_cast<Ogre::Entity*>(shaftObj))
      ((Ogre::Entity*)shaftObj)->setMaterialName(_shaftColor.c_str());
    if (dynamic_cast<Ogre::Entity*>(headObj))
      ((Ogre::Entity*)headObj)->setMaterialName(_headColor.c_str());

	// initialize SceneNode, second node is connected to the first one
	this->arrowShaftNodes.push_back(
			this->mSceneMgr->getRootSceneNode()->createChildSceneNode());
	this->arrowHeadNodes.push_back(this->arrowShaftNodes.back()->createChildSceneNode());

	// Attach object to the Scene Node
	this->arrowShaftNodes.back()->attachObject(shaftObj);
	this->arrowHeadNodes.back()->attachObject(headObj);

	// set position / orientation of the nodes, the arrow head is offseted
	this->arrowShaftNodes.back()->setPosition(_pos);
	this->arrowHeadNodes.back()->setPosition(0, 0, 0.1);

	this->arrowShaftNodes.back()->setOrientation(
			Ogre::Quaternion(1, 1, 0, GZ_DTOR(90)));

	// set the scale of the arrow
	this->arrowShaftNodes.back()->setScale(_scale);

	// set the arrow to visible
	this->arrowShaftNodes.back()->setVisible(true);
	this->arrowHeadNodes.back()->setVisible(true);
}

//////////////////////////////////////////////////
void TrajectoryFactorySystemPlugin::CreateMeshEntity()
{
	// Init Visual ptr
	this->meshVisPtr =  boost::make_shared<rendering::Visual>
						(rendering::Visual("mesh_visual", this->mScene, false));


	// Add hand meshes to the resources
	this->meshVisPtr->InsertMesh(
			"model://hit_hand/meshes/hand/right_palm.dae");
	this->meshVisPtr->InsertMesh(
			"model://hit_hand/meshes/hand/finger_base.dae");
	this->meshVisPtr->InsertMesh(
			"model://hit_hand/meshes/hand/finger_proximal.dae");
	this->meshVisPtr->InsertMesh(
			"model://hit_hand/meshes/hand/finger_middle.dae");
	this->meshVisPtr->InsertMesh(
			"model://hit_hand/meshes/hand/finger_distal.dae");
	this->meshVisPtr->InsertMesh(
			"model://hit_hand/meshes/hand/right_thumb_base.dae");



	// Create objects after the meshes
	Ogre::MovableObject* palmObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"hand_palm", "model://hit_hand/meshes/hand/right_palm.dae"));
	Ogre::MovableObject* thumbBaseObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"thumb_base", "model://hit_hand/meshes/hand/right_thumb_base.dae"));

	Ogre::MovableObject* foreFingerBaseObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"fore_finger_base", "model://hit_hand/meshes/hand/finger_base.dae"));
	Ogre::MovableObject* foreFingerProximalObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"fore_finger_proximal", "model://hit_hand/meshes/hand/finger_proximal.dae"));
	Ogre::MovableObject* foreFingerMiddleObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"fore_finger_middle", "model://hit_hand/meshes/hand/finger_middle.dae"));
	Ogre::MovableObject* foreFingerDistalObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"fore_finger_distal", "model://hit_hand/meshes/hand/finger_distal.dae"));

	Ogre::MovableObject* middleFingerBaseObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"middle_finger_base", "model://hit_hand/meshes/hand/finger_base.dae"));
	Ogre::MovableObject* middleFingerProximalObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"middle_finger_proximal", "model://hit_hand/meshes/hand/finger_proximal.dae"));
	Ogre::MovableObject* middleFingerMiddleObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"middle_finger_middle", "model://hit_hand/meshes/hand/finger_middle.dae"));
	Ogre::MovableObject* middleFingerDistalObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"middle_finger_distal", "model://hit_hand/meshes/hand/finger_distal.dae"));

	Ogre::MovableObject* ringFingerBaseObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"ring_finger_base", "model://hit_hand/meshes/hand/finger_base.dae"));
	Ogre::MovableObject* ringFingerProximalObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"ring_finger_proximal", "model://hit_hand/meshes/hand/finger_proximal.dae"));
	Ogre::MovableObject* ringFingerMiddleObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"ring_finger_middle", "model://hit_hand/meshes/hand/finger_middle.dae"));
	Ogre::MovableObject* ringFingerDistalObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"ring_finger_distal", "model://hit_hand/meshes/hand/finger_distal.dae"));

	Ogre::MovableObject* thumbFingerBaseObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"thumb_finger_base", "model://hit_hand/meshes/hand/finger_base.dae"));
	Ogre::MovableObject* thumbFingerProximalObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"thumb_finger_proximal", "model://hit_hand/meshes/hand/finger_proximal.dae"));
	Ogre::MovableObject* thumbFingerMiddleObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"thumb_finger_middle", "model://hit_hand/meshes/hand/finger_middle.dae"));
	Ogre::MovableObject* thumbFingerDistalObj =
			(Ogre::MovableObject*) (this->mSceneMgr->createEntity(
    	    		"thumb_finger_distal", "model://hit_hand/meshes/hand/finger_distal.dae"));



	// initialize SceneNodes, palm being the root, the rest the children
	this->palmNode = this->mSceneMgr->getRootSceneNode()->createChildSceneNode();
	this->foreFingerBaseNode = this->palmNode->createChildSceneNode();
	this->foreFingerProximalNode = this->palmNode->createChildSceneNode();
	this->foreFingerMiddleNode = this->palmNode->createChildSceneNode();
	this->foreFingerDistalNode = this->palmNode->createChildSceneNode();

	this->middleFingerBaseNode = this->palmNode->createChildSceneNode();
	this->middleFingerProximalNode = this->palmNode->createChildSceneNode();
	this->middleFingerMiddleNode = this->palmNode->createChildSceneNode();
	this->middleFingerDistalNode = this->palmNode->createChildSceneNode();

	this->ringFingerBaseNode = this->palmNode->createChildSceneNode();
	this->ringFingerProximalNode = this->palmNode->createChildSceneNode();
	this->ringFingerMiddleNode = this->palmNode->createChildSceneNode();
	this->ringFingerDistalNode = this->palmNode->createChildSceneNode();

	this->thumbBaseNode = this->palmNode->createChildSceneNode();
	this->thumbFingerBaseNode = this->palmNode->createChildSceneNode();
	this->thumbFingerProximalNode = this->palmNode->createChildSceneNode();
	this->thumbFingerMiddleNode = this->palmNode->createChildSceneNode();
	this->thumbFingerDistalNode = this->palmNode->createChildSceneNode();



	// Attach objects to the SceneNodes
	this->palmNode->attachObject(palmObj);
	this->foreFingerBaseNode->attachObject(foreFingerBaseObj);
	this->foreFingerProximalNode->attachObject(foreFingerProximalObj);
	this->foreFingerMiddleNode->attachObject(foreFingerMiddleObj);
	this->foreFingerDistalNode->attachObject(foreFingerDistalObj);

	this->middleFingerBaseNode->attachObject(middleFingerBaseObj);
	this->middleFingerProximalNode->attachObject(middleFingerProximalObj);
	this->middleFingerMiddleNode->attachObject(middleFingerMiddleObj);
	this->middleFingerDistalNode->attachObject(middleFingerDistalObj);

	this->ringFingerBaseNode->attachObject(ringFingerBaseObj);
	this->ringFingerProximalNode->attachObject(ringFingerProximalObj);
	this->ringFingerMiddleNode->attachObject(ringFingerMiddleObj);
	this->ringFingerDistalNode->attachObject(ringFingerDistalObj);

	this->thumbBaseNode->attachObject(thumbBaseObj);
	this->thumbFingerBaseNode->attachObject(thumbFingerBaseObj);
	this->thumbFingerProximalNode->attachObject(thumbFingerProximalObj);
	this->thumbFingerMiddleNode->attachObject(thumbFingerMiddleObj);
	this->thumbFingerDistalNode->attachObject(thumbFingerDistalObj);





	// Set position of the parent mesh (palm)
	this->palmNode->setPosition(0,-0.5,1);
	// Set relative position / orientation of the children meshes
	// positions are hard-coded from the SDF file
	this->foreFingerBaseNode->setPosition(-0.008300, 0.040165, 0.145450);
	this->foreFingerBaseNode->setOrientation(this->RPYToOgreQuat(
			-1.570800, -1.536490, -1.570800));
	this->foreFingerProximalNode->setPosition(-0.004300, 0.040165, 0.145450);
	this->foreFingerProximalNode->setOrientation(this->RPYToOgreQuat(
			0.000000, -1.536490, -1.570800));
	this->foreFingerMiddleNode->setPosition(-0.004300, 0.037839, 0.213210);
	this->foreFingerMiddleNode->setOrientation(this->RPYToOgreQuat(
			0.000000, -1.536490, -1.570800));
	this->foreFingerDistalNode->setPosition(-0.004300, 0.036810, 0.243192);
	this->foreFingerDistalNode->setOrientation(this->RPYToOgreQuat(
			1.536490, -0.000796, -3.141560));

	this->middleFingerBaseNode->setPosition(-0.004300, 0.000000, 0.150150);
	this->middleFingerBaseNode->setOrientation(this->RPYToOgreQuat(
			-0.000003, -1.570793, 3.141593));
	this->middleFingerProximalNode->setPosition(-0.004300, 0.000000, 0.150150);
	this->middleFingerProximalNode->setOrientation(this->RPYToOgreQuat(
			1.570793, -1.570793, 3.141593));
	this->middleFingerMiddleNode->setPosition(-0.004300, 0.000000, 0.217950);
	this->middleFingerMiddleNode->setOrientation(this->RPYToOgreQuat(
			1.570793, -1.570793, 3.141593));
	this->middleFingerDistalNode->setPosition(-0.004300, 0.000000, 0.247950);
	this->middleFingerDistalNode->setOrientation(this->RPYToOgreQuat(
			1.570800, -0.000797, 3.141590));

	this->ringFingerBaseNode->setPosition(-0.004300, -0.040165, 0.145430);
	this->ringFingerBaseNode->setOrientation(this->RPYToOgreQuat(
			1.570800, -1.536490, 1.570800));
	this->ringFingerProximalNode->setPosition(-0.004300, -0.040165, 0.145430);
	this->ringFingerProximalNode->setOrientation(this->RPYToOgreQuat(
			3.141590, -1.536490, 1.570800));
	this->ringFingerMiddleNode->setPosition(-0.004300, -0.037839, 0.213190);
	this->ringFingerMiddleNode->setOrientation(this->RPYToOgreQuat(
			3.141590, -1.536490, 1.570800));
	this->ringFingerDistalNode->setPosition(-0.004300, -0.036810, 0.243172);
	this->ringFingerDistalNode->setOrientation(this->RPYToOgreQuat(
			1.605100, -0.000796, 3.141560));

	this->thumbBaseNode->setPosition(-0.003000, 0.027100, 0.000000);
	this->thumbBaseNode->setOrientation(this->RPYToOgreQuat(
			0.000000, 0.000000, 0.000000));
	this->thumbFingerBaseNode->setPosition(-0.009000, 0.114000, 0.097000);
	this->thumbFingerBaseNode->setOrientation(this->RPYToOgreQuat(
			-0.000000, -0.959928, 1.570800));
	this->thumbFingerProximalNode->setPosition(-0.009000, 0.114000, 0.097000);
	this->thumbFingerProximalNode->setOrientation(this->RPYToOgreQuat(
			1.570800, -0.959928, 1.570800));
	this->thumbFingerMiddleNode->setPosition(-0.009000, 0.152889, 0.152538);
	this->thumbFingerMiddleNode->setOrientation(this->RPYToOgreQuat(
			1.570800, -0.959928, 1.570800));
	this->thumbFingerDistalNode->setPosition(-0.009000, 0.170096, 0.177113);
	this->thumbFingerDistalNode->setOrientation(this->RPYToOgreQuat(
			1.570800, 0.610073, 1.570800));



	this->palmNode->setVisible(true);
	this->foreFingerBaseNode->setVisible(true);
	this->foreFingerProximalNode->setVisible(true);
	this->foreFingerMiddleNode->setVisible(true);
	this->foreFingerDistalNode->setVisible(true);

	this->middleFingerBaseNode->setVisible(true);
	this->middleFingerProximalNode->setVisible(true);
	this->middleFingerMiddleNode->setVisible(true);
	this->middleFingerDistalNode->setVisible(true);

	this->ringFingerBaseNode->setVisible(true);
	this->ringFingerProximalNode->setVisible(true);
	this->ringFingerMiddleNode->setVisible(true);
	this->ringFingerDistalNode->setVisible(true);

	this->thumbBaseNode->setVisible(true);
	this->thumbFingerBaseNode->setVisible(true);
	this->thumbFingerProximalNode->setVisible(true);
	this->thumbFingerMiddleNode->setVisible(true);
	this->foreFingerDistalNode->setVisible(true);
}

//////////////////////////////////////////////////
Ogre::Quaternion TrajectoryFactorySystemPlugin::RPYToOgreQuat(
		double _r, double _p, double _y)
{
	math::Quaternion gazebo_quat = math::Quaternion(math::Vector3(_r, _p, _y));

	return Ogre::Quaternion(
			gazebo_quat.w, gazebo_quat.x, gazebo_quat.y, gazebo_quat.z);
}

//////////////////////////////////////////////////
void TrajectoryFactorySystemPlugin::SetInterpolationSplines(
		const std::vector<math::Pose> _pose_vector,
		math::Spline &_pos_spline,
		math::RotationSpline &_rot_spline)
{
	// TODO what are this?
	_pos_spline.SetAutoCalculate(false);
	_rot_spline.SetAutoCalculate(false);

	_pos_spline.Clear();
	_rot_spline.Clear();

	for (unsigned int i = 0; i < _pose_vector.size(); i++)
	{
		_pos_spline.AddPoint(_pose_vector[i].pos);
		_rot_spline.AddPoint(_pose_vector[i].rot);
	}

	_pos_spline.RecalcTangents();
	_rot_spline.RecalcTangents();
}

//////////////////////////////////////////////////
math::Pose TrajectoryFactorySystemPlugin::GetPoseFromSpline(
		double _frame,
		math::Spline &_pos_spline,
		math::RotationSpline &_rot_spline)
{
	return math::Pose(
			_pos_spline.Interpolate(_frame),
			_rot_spline.Interpolate(_frame,
			false));//TODO what are this?
}

//////////////////////////////////////////////////
std::vector<math::Pose> TrajectoryFactorySystemPlugin::GetTrajectoryFromFile(std::string _filename)
{
	// the file to open
	std::ifstream infile;
	std::vector<double> line_values;
	std::vector<math::Pose> trajectory;

	// open the file
	infile.open(_filename.c_str(), std::ifstream::in);
	std::cout<<"Reading trajectory from "<< _filename.c_str() << std::endl;


	if (infile.is_open())
	{
		while (infile.good())
		{
			// read the entire line into a string
			std::string line;
			getline(infile, line);

			// now we'll use a stringstream to separate the fields out of the line
			std::stringstream ss( line );
			std::string field;
			while (getline( ss, field, ' ' ))
			{
				// for each field we wish to convert it to a double
				std::stringstream d_s( field );
				double d_value = 0.0;  // (default value is 0.0)
				d_s >> d_value;

				line_values.push_back(d_value);
			}
			/*0 - timestamp; 1-2-3 XYZ; 4-5-6 RPY*/
			// append Pose to the trajectory
			trajectory.push_back(math::Pose(
					math::Vector3(line_values[1],line_values[2],line_values[3]), //position X Y Z
					math::Quaternion(line_values[4],line_values[5],line_values[6]))); //rotation R P Y

			// clear the line vector
			line_values.clear();

		}
	}
	return trajectory;
}

GZ_REGISTER_SYSTEM_PLUGIN(TrajectoryFactorySystemPlugin)



















