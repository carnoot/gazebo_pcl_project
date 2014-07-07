#include "gazebo/gui/GuiIface.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/gazebo.hh"

namespace gazebo {

/// \SystemGui Class
	class SystemGUI : public SystemPlugin {

		/// \ Constructor
	public: virtual ~SystemGUI();

/// \Load

	public: virtual void Load(int /*_argc*/, char ** /*_argv*/);

/// \Init

	private: virtual void Init();

	/// \userCam
	private: rendering::UserCameraPtr userCam;

	/// \connections
	private: std::vector<event::ConnectionPtr> connections;

	};

}

