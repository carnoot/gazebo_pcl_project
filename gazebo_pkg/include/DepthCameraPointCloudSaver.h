#include <boost/bind.hpp>

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include <gazebo/common/common.hh>
#include "gazebo/gazebo.hh"
#include "gazebo_pkg/GetTextParam.h"
#include "ros/ros.h"
#include "/opt/ros/hydro/include/pcl_conversions/pcl_conversions.h"
#include "pcl/io/png_io.h"

namespace gazebo {

/// \SystemGui Class
class DepthCameraPointCloudSaver {

	/// \ Constructor
public:
	DepthCameraPointCloudSaver();

public:
	virtual ~DepthCameraPointCloudSaver();
public:
	void delayFunction();

public:
	void customCreateText(gazebo_pkg::GetTextParam::Request textParam);
	std::string convertFloat(float _float);
	void initSubscriber();
	void callback(const sensor_msgs::PointCloud2::ConstPtr &message);

public:
	ros::ServiceServer service;
	ros::Subscriber sub;
	ros::AsyncSpinner *spinner;
	float numberOfCalls;
	std::vector<sensor_msgs::PointCloud2> pointCloudVector;
	pcl::PointCloud<pcl::PointXYZRGB> myPointCloud;

private:
	Ogre::Timer *timer;
	gazebo_pkg::GetTextParam::Request textRequest;
	bool called;
};

}

