#include <boost/bind.hpp>
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include <gazebo/common/common.hh>

namespace gazebo {

	class SystemGUI : public SystemPlugin {

	public: SystemGUI();
	public: virtual ~SystemGUI();

	public: virtual void Load(int /*_argc*/, char ** /*_argv*/);

	private: virtual void Init();

	private: void OnUpdate();

	private: rendering::UserCameraPtr userCam;

	private: std::vector<event::ConnectionPtr> connections;
	public: event::ConnectionPtr updateConnection;
	public: int contor;

	};

}

