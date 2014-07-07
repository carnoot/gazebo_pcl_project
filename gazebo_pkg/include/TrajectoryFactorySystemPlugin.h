#ifndef TRAJECTORY_FACTORY_SYSTEM_PLUGIN_HH
#define TRAJECTORY_FACTORY_SYSTEM_PLUGIN_HH

#include "gazebo/gazebo.hh"
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include "gazebo/rendering/ArrowVisual.hh"
#include "gazebo/common/CommonIface.hh"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include <OGRE/Ogre.h>
#include "OGRE/OgreSceneNode.h"
#include "OGRE/OgreEntity.h"
#include "OGRE/OgreSubEntity.h"
#include "OGRE/OgreSceneManager.h"

#include <fstream>
#include <iostream>
#include <sstream>

namespace gazebo
{

	typedef std::pair<math::Spline, math::RotationSpline> PoseSplinePair;


	/// \brief Class representing a trajectory
	class Trajectory
	{
		/// \brief Constructor
		public: Trajectory()
		{
			this->finished_drawing = false;
		}

		/// \brief Get the Trajectory from a csv file
		public: void SetPosesFromFile(std::string _filename)
		{
			// clear the last poses
			this->mPoses.clear();

			// the file to open
			std::ifstream infile;
			std::vector<double> line_values;

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
					this->mPoses.push_back(math::Pose(
							math::Vector3(line_values[1],line_values[2],line_values[3]), //position X Y Z
							math::Quaternion(line_values[4],line_values[5],line_values[6]))); //rotation R P Y

					// clear the line vector
					line_values.clear();
				}
			}

		}

		/// \brief Set the Poses
		public: void SetPoses(std::vector<math::Pose> _poses)
		{
			for (unsigned int i = 0; i<_poses.size(); i++)
			{
				this->mPoses.push_back(_poses[i]);
			}
		}

		/// \brief Get the Poses
		public: std::vector<math::Pose> GetPoses()
		{
			return this->mPoses;
		}

		/// \brief create all Ogre Entities with the given color
		public: void SetEntities(Ogre::SceneManager* _sceneMgr, std::string _color)
		{
			for (unsigned int i = 0; i< this->mPoses.size(); i++)
			{
				this->mEntities.push_back(_sceneMgr->createEntity(
						Ogre::SceneManager::PT_SPHERE));

				// set material
				this->mEntities.back()->setMaterialName(_color.c_str());
			}
		}

		/// \brief Set the SceneNodes attached to the class Entities and Poses
		public: void SetSceneNodes(Ogre::SceneManager* _sceneMgr, float _scale,
				bool _visible)
		{
			Ogre::Vector3 ogre_vec3;
			Ogre::Quaternion ogre_quat;

			for (unsigned int i = 0; i< this->mPoses.size(); i++)
			{
				// init Scene Node
				this->mSceneNodes.push_back(_sceneMgr->getRootSceneNode(
						)->createChildSceneNode());
				// attach entity
				this->mSceneNodes.back()->attachObject(this->mEntities[i]);

				// set scale
				this->mSceneNodes.back()->setScale(_scale, _scale, _scale);

				// set position
				ogre_vec3 = Ogre::Vector3(this->mPoses[i].pos.x,
										this->mPoses[i].pos.y,
										this->mPoses[i].pos.z);

				this->mSceneNodes.back()->setPosition(ogre_vec3);

				// set orientation
				ogre_quat = Ogre::Quaternion(this->mPoses[i].rot.w,
											this->mPoses[i].rot.x,
											this->mPoses[i].rot.y,
											this->mPoses[i].rot.z);

				this->mSceneNodes.back()->setOrientation(ogre_quat);

				// set visibility
				this->mSceneNodes.back()->setVisible(_visible);
			}
		}

		/// \brief Set the SceneNodes attached to the class Entities and Poses
		public: void SetLinePoints(Ogre::SceneManager* _sceneMgr,
								std::string _color,
								bool _visible)
		{
			// create the scnene node for the line
			this->mLineSceneNode = _sceneMgr->getRootSceneNode(
					)->createChildSceneNode();

			// create the line object
			this->mLineObj = _sceneMgr->createManualObject();

			// set visibility
			this->mLineSceneNode->setVisible(_visible);
			this->mLineObj->setVisible(_visible);

			this->mLineObj->clear();
			this->mLineObj->begin(_color.c_str(), Ogre::RenderOperation::OT_LINE_LIST);
			// add the points for the lines
			for (unsigned int i = 1; i < this->mPoses.size()-1; i++)
			{
				// start point
				this->mLineObj->position(this->mPoses[i].pos.x,
						this->mPoses[i].pos.y,
						this->mPoses[i].pos.z);
				// end point
				this->mLineObj->position(this->mPoses[i+1].pos.x,
						this->mPoses[i+1].pos.y,
						this->mPoses[i+1].pos.z);
			}
			this->mLineObj->end();

			// attach line object to scene node
			this->mLineSceneNode->attachObject(this->mLineObj);
		}

		/// \brief Set visual color and visibility
		public: bool VisualizeFrame(unsigned int _frame,
									std::string _color,
									bool _visible,
									float _scale)
		{
			if (_frame < this->mPoses.size())
			{
				// set material
				this->mEntities.at(_frame)->setMaterialName(_color.c_str());
				// set scale
				this->mSceneNodes.at(_frame)->setScale(_scale, _scale, _scale);
				// set visibility
				this->mSceneNodes.at(_frame)->setVisible(_visible);

				return true;
			}
			else
			{
				std::cout << " Last frame visualized, drawing finished = true " << std::endl;
				// set the flag that the trajectory is drawn
				this->finished_drawing = true;
				return false;
			}
		}

		/// \brief the poses of the trajectory
		private: std::vector<math::Pose> mPoses;

		/// \brief the ogre Entities of the trajectory
		private: std::vector<Ogre::Entity*> mEntities;

		/// \brief the ogre SceneNodes of the trajectory
		private: std::vector<Ogre::SceneNode*> mSceneNodes;

		/// \brief the ogre Line Object of the trajectory
		private: Ogre::ManualObject* mLineObj;

		/// \brief the ogre Scene Node for the line of the trajectory
		private: Ogre::SceneNode* mLineSceneNode;

		/// \brief flag that the trajectory is finished drawing
		public: bool finished_drawing;

	};




	/// \brief System Plugin for visualizing trajectories
	class TrajectoryFactorySystemPlugin : public SystemPlugin
	{
	/// \brief Constructor
	public: TrajectoryFactorySystemPlugin();

	/// \brief Destructor
	public: virtual ~TrajectoryFactorySystemPlugin();

	/// \brief Load method
	protected: virtual void Load(int /*_argc*/, char ** /*_argv*/);

	/// \brief Init method
	protected: virtual void Init();

	/// \brief OnUpdate method called at every PreRender event
	protected: virtual void OnUpdate();

	/// \brief Called by the updateThread
	private: void DrawTrajectories(void);

	/// \brief Get an OGRE entity
	private: Ogre::Entity* CreateEntity(std::string _color);

	/// \brief Get an OGRE entity
	private: Ogre::SceneNode* CreateSceneNode(
			Ogre::Entity &_entity,
			float _scale,
			Ogre::Vector3 _pos);

	/// \brief Set the material of the given entity to transparent
	private: void SetTransparency(Ogre::Entity* _entity, float _transparency);

	/// \brief Create an arrow visualization
	private: void CreateArrowEntity(
			std::string _headColor,
			std::string _shaftColor,
			Ogre::Vector3 _pos,
			Ogre::Quaternion _orientation,
			Ogre::Vector3 _scale);

	/// \brief Create an OGRE mesh visualization (Hit Hand)
	private: void CreateMeshEntity();

	/// \brief Change from euler angles to OGRE quaternion
	private: Ogre::Quaternion RPYToOgreQuat(double _r, double _p, double _y);

	/// \brief Set an interpolation spline and rotation spline from a vector of poses
	private: void SetInterpolationSplines(
			const std::vector<math::Pose> _pose_vector,
			math::Spline &_pos_spline,
			math::RotationSpline &_rot_spline);

	/// \brief Get the interpolated pose at the given frame
	private: math::Pose GetPoseFromSpline(
			double _frame,
			math::Spline &_pos_spline,
			math::RotationSpline &_rot_spline);

	/// \brief Get the Trajectory from a csv file
	private: std::vector<math::Pose> GetTrajectoryFromFile(std::string _filename);

	/// \brief update connection pointer
	private: event::ConnectionPtr updateConnection;

	/// \brief update thread
	private: boost::thread *updateThread;

	/// \brief Ogre scene pointer
	private: rendering::ScenePtr mScene;

	/// \brief ogre scene manager
	private: Ogre::SceneManager *mSceneMgr;

	/// \brief vector with all trajectories
	private: std::vector<Trajectory> mugTrajectories, spatulaTrajectories;


	/// \brief Vector of position and rotation spline pairs
	private: std::vector<PoseSplinePair> poseSplines;

	/// \brief Vector of vector of poses, pose trajectory used to create the splines
	private: std::vector<std::vector<math::Pose> > poseTrajectories;

	/// \brief Vector of vector Ogre Entites  for each trajectory
	private: std::vector< std::vector<Ogre::Entity*> > entityTrajectories;

	/// \brief Vector of vector Ogre Entites  for each trajectory
	private: std::vector< std::vector<Ogre::SceneNode*> > sceneNodeTrajectories;


	private: std::vector<Ogre::Entity*> sphereEntities;

	private: std::vector<Ogre::SceneNode*> sphereNodes;

	private: Ogre::Vector3 pos;

	private: rendering::VisualPtr arrowVisPtr;

	private: std::vector<Ogre::Entity*> arrowHeadEntities;

	private: std::vector<Ogre::Entity*> arrowShaftEntities;

    private: std::vector<Ogre::SceneNode*> arrowHeadNodes;

    private: std::vector<Ogre::SceneNode*> arrowShaftNodes;


    /// \brief interpolation for position
    private: math::Spline *positionSpline;

    /// \brief interpolation for rotation
    private: math::RotationSpline *rotationSpline;

    /// \brief pose vector to interpolate
    private: std::vector<math::Pose> trajectory;

    /// \brief frame to visualize
    private: double frame;



	private: rendering::VisualPtr meshVisPtr;

    private: const common::Mesh* mesh;

    /// \brief Hit Hand mesh scene nodes
    private: Ogre::SceneNode *palmNode,
							 *foreFingerBaseNode,
							 *foreFingerProximalNode,
							 *foreFingerMiddleNode,
							 *foreFingerDistalNode,
							 *middleFingerBaseNode,
							 *middleFingerProximalNode,
							 *middleFingerMiddleNode,
							 *middleFingerDistalNode,
							 *ringFingerBaseNode,
							 *ringFingerProximalNode,
							 *ringFingerMiddleNode,
							 *ringFingerDistalNode,
							 *thumbBaseNode,
							 *thumbFingerBaseNode,
							 *thumbFingerProximalNode,
							 *thumbFingerMiddleNode,
							 *thumbFingerDistalNode;


	};
}

#endif
