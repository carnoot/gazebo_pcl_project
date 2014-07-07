#include "gazebo/gui/GuiIface.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "ros/ros.h"

namespace gazebo {

/// \SystemGui Class
	class DrawTextPlugin : public SystemPlugin {

		/// \ Constructor
	public:	DrawTextPlugin();

	public: virtual ~DrawTextPlugin();

/// \Load

	public: virtual void Load(int /*_argc*/, char ** /*_argv*/);

/// \Init

	private: virtual void Init();

	public:
		rendering::ScenePtr scene;
		Ogre::SceneManager *sceneManager;
		Ogre::SceneNode *node;
		Ogre::Entity *ent;

	private:
		rendering::MovableText* text;
	    rendering::UserCameraPtr userCam;
		gazebo::math::Vector3 *vectstart;
		math::Vector3 *vectend;
	};

}

