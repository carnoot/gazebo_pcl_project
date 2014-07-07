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
	class DrawLinesPlugin : public SystemPlugin {

		/// \ Constructor
	public:	DrawLinesPlugin();

	public: virtual ~DrawLinesPlugin();

/// \Load

	public: virtual void Load(int /*_argc*/, char ** /*_argv*/);

/// \Init

	private: virtual void Init();

	public:
		rendering::ScenePtr scene;

	private:
	    rendering::UserCameraPtr userCam;
		gazebo::math::Vector3 *vectstart;
		math::Vector3 *vectend;
	};

}

