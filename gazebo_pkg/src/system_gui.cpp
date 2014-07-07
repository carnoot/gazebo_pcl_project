#include "gazebo/gui/GuiIface.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/gazebo.hh"
#include "system_gui.h"


using namespace gazebo;

    SystemGUI::~SystemGUI()
    {
      if (this->userCam)
        this->userCam->EnableSaveFrame(false);
    }

    void SystemGUI::Load(int /*_argc*/, char ** /*_argv*/)
    {
    }

    void SystemGUI::Init()
    {
      // Get a pointer to the active user camera
      this->userCam = gui::get_active_camera();

      // Enable saving frames
      this->userCam->EnableSaveFrame(true);

      // Specify the path to save frames into
      this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");
    }

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
