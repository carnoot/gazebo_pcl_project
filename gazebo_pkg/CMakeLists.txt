cmake_minimum_required(VERSION 2.8.3)
project(gazebo_pkg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x  -Wno-deprecated -DBT_USE_DOUBLE_PRECISION")
set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(VTK_DIR "/usr/include/vtk-5.8")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL REQUIRED)

find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(catkin REQUIRED
	roscpp
	rospy
	std_msgs
	geometry_msgs
	message_generation
	urdf
	kdl_parser
	robot_state_publisher
	cv_bridge
	gazebo_plugins
	eigen
)

add_message_files(
	FILES
	Object.msg
	SherpaWorldObject.msg
)

add_service_files(
	FILES
	GetObject.srv
	AddTwoInts2.srv
	GetCoordFromClient.srv
	GetTextParam.srv
	SendStampedPose.srv
	ObjectInspectionNumber.srv
	ObjectInspectionBounding.srv
	ObjectInspectionCenter.srv
)

generate_messages(   
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###########GAZEBO######################
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(SDF sdformat REQUIRED)
endif()
include_directories(include ${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${SDF_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs /usr/include/vtk-5.8)
link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(custom_pose_message/msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_pkg 
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs gazebo_plugins pcl_ros pcl
  DEPENDS system_lib gazebo
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(
	include
	${catkin_INCLUDE_DIRS} ${gazebo_pkg_SOURCE_DIR}/include
)

## HACK PENTRU QT ( DE LA FERKO ) ##
execute_process(COMMAND find ${PROJECT_SOURCE_DIR}  -path "${PROJECT_SOURCE_DIR}/include/*" -name "*.h*" -or -name "*.cpp" OUTPUT_VARIABLE RS_FILES_STRING)
 #cmake string to list; has to be done because otherwise #add_custom_target would handle    RS_FILES_STRING as one single #file, which leads to an error
 string(REPLACE "\n" ";" RS_FILES_LIST ${RS_FILES_STRING})
 add_custom_target(project_additional_files
   SOURCES
   ${RS_FILES_LIST}
 )

file(GLOB SOURCES ~/catkin_ws/src/gazebo_ros_pkgs/gazebo_plugins/src/gazebo_ros_openni_kinect_test.cpp)

## Declare a cpp executable
#add_library(test_plugin src/test_plugin.cpp)
#add_library(model_push src/model_push.cpp)
#add_library(factory src/factory.cpp)
#add_library(world_edit src/world_edit.cpp)
#add_library(system_gui src/system_gui.cpp)
add_library(ros_plugin src/ros_plugin.cpp)
#add_library(ros_plugin_service src/ros_plugin_service.cpp)
add_library(ros_get_coord src/ros_get_coord.cpp)
#add_library(DynamicJoint SHARED src/DynamicJoint.cpp)
#add_library(VisualPluginTest src/VisualPluginTest.cpp)
#add_library(DrawLinesPlugin src/DrawLinesPlugin.cpp)
#add_library(DrawTextPlugin src/DrawTextPlugin.cpp)
#add_library(TrajectoryFactorySystemPlugin src/TrajectoryFactorySystemPlugin.cpp)
add_library(OgreTest src/OgreTest.cpp)
#add_library(KeyPressEventExampleFromGazebo src/KeyPressEventExampleFromGazebo.cpp)
#add_library(AddTextGazebo src/AddTextGazebo.cpp)
#add_library(SystemPluginArgs src/SystemPluginArgs.cpp)
#add_library(factory SHARED /home/furdek/gazebo_source_1.9.5/gazebo/plugins/DepthCameraPlugin.cc)
#add_library(factory SHARED /home/furdek/catkin_ws/src/gazebo_pkg/src/gazebo_ros_openni_kinect.cpp)
#add_library(gazebo_ros_openni_kinect ~/catkin_ws/src/gazebo_pkg/src/gazebo_ros_openni_kinect.cpp)
#add_library(factory /home/furdek/catkin_ws/src/gazebo_pkg/src/gazebo_ros_openni_kinect.cpp)
#add_library(factory SHARED /home/furdek/catkin_ws/src/gazebo_pkg/src/MoveKinectCamera.cpp)
add_library(gazebo_ros_openni_kinect_test ~/catkin_ws/src/gazebo_ros_pkgs/gazebo_plugins/src/gazebo_ros_openni_kinect_test.cpp)

add_library(MoveKinectCamera src/MoveKinectCamera.cpp)
add_library(MoveKinectCameraPlugin src/MoveKinectCameraPlugin.cpp)
add_library(KitchenFinalPlugin src/KitchenFinalPlugin.cpp)
#add_library(SherpaObjectModelPlugin src/ModelPluginForBenjamin.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(test_plugin ${catkin_LIBRARIES})
#target_link_libraries(model_push ${catkin_LIBRARIES})
#target_link_libraries(factory ${catkin_LIBRARIES})
#target_link_libraries(world_edit ${catkin_LIBRARIES})
#target_link_libraries(system_gui ${catkin_LIBRARIES} ${OGRE_LIBRARIES})
target_link_libraries(ros_plugin ${catkin_LIBRARIES})
#target_link_libraries(ros_plugin_service ${catkin_LIBRARIES})
target_link_libraries(ros_get_coord ${catkin_LIBRARIES})
#target_link_libraries(DynamicJoint ${catkin_LIBRARIES})
#target_link_libraries(VisualPluginTest ${catkin_LIBRARIES})
#target_link_libraries(DrawLinesPlugin ${catkin_LIBRARIES} ${OGRE_LIBRARIES})
#target_link_libraries(DrawTextPlugin ${catkin_LIBRARIES} ${OGRE_LIBRARIES})
target_link_libraries(OgreTest ${catkin_LIBRARIES} ${OGRE_LIBRARIES})
#target_link_libraries(KeyPressEventExampleFromGazebo ${catkin_LIBRARIES} ${OGRE_LIBRARIES})
#target_link_libraries(TrajectoryFactorySystemPlugin ${catkin_LIBRARIES} ${OGRE_LIBRARIES})
#target_link_libraries(AddTextGazebo ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(SystemPluginArgs ${catkin_LIBRARIES} ${OGRE_LIBRARIES})
#target_link_libraries(gazebo_ros_openni_kinect ${catkin_LIBRARIES} ${SDF_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(factory ${GAZEBO_LIBRARIES})
target_link_libraries(KitchenFinalPlugin ${catkin_LIBRARIES} ${kdl_parser_LIBRARIES})
#target_link_libraries(SherpaObjectModelPlugin ${catkin_LIBRARIES} ${SDF_LIBRARIES})

target_link_libraries(MoveKinectCamera custom_pose_message ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARY})
target_link_libraries(MoveKinectCameraPlugin custom_pose_message ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARY})

target_link_libraries(gazebo_ros_openni_kinect_test ${catkin_LIBRARIES})

#add_executable(add_two_ints_client src/add_two_ints_client.cpp)
#target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})

#add_executable(add_coord_client src/add_coord_client.cpp)
#target_link_libraries(add_coord_client ${catkin_LIBRARIES})

add_executable(add_object_client src/add_object_client.cpp)
target_link_libraries(add_object_client ${catkin_LIBRARIES})

add_executable(add_text_client src/add_text_client.cpp)
target_link_libraries(add_text_client ${catkin_LIBRARIES})

add_executable(add_text_publisher_client src/add_text_publisher_client.cpp)
target_link_libraries(add_text_publisher_client ${catkin_LIBRARIES})

add_executable(MoveKinectCameraClient src/MoveKinectCameraClient.cpp)
target_link_libraries(MoveKinectCameraClient ${catkin_LIBRARIES})

add_executable(DepthCameraPointCloudSaver src/DepthCameraPointCloudSaver.cpp)
target_link_libraries(DepthCameraPointCloudSaver ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(RGBImageSaver src/RGBImageSaver.cpp)
#target_link_libraries(RGBImageSaver ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(PlanarSegmentation src/PlanarSegmentation.cpp)
target_link_libraries(PlanarSegmentation ${PCL_LIBRARIES})

add_executable(ExtractingIndices src/ExtractingIndices.cpp)
target_link_libraries(ExtractingIndices ${PCL_LIBRARIES})

add_executable(ClusterExtraction src/ClusterExtraction.cpp)
target_link_libraries(ClusterExtraction ${PCL_LIBRARIES})

add_executable(ConcaveHullPolygon src/ConcaveHullPolygon.cpp)
target_link_libraries(ConcaveHullPolygon ${PCL_LIBRARIES})

add_executable(JointStatePublisherTry src/JointStatePublisherTry.cpp)
target_link_libraries(JointStatePublisherTry ${catkin_LIBRARIES})

add_executable(DepthImageRawToCvMat src/DepthImageRawToCvMat.cpp)
target_link_libraries(DepthImageRawToCvMat ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(OpenPCDAndCompare src/OpenPCDAndCompare.cpp)
target_link_libraries(OpenPCDAndCompare ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(EuclideanClusterProba0 src/EuclideanClusterProba0.cpp)
target_link_libraries(EuclideanClusterProba0 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(MultiPlaneSegmenter src/MultiPlaneSegmenter.cpp)
target_link_libraries(MultiPlaneSegmenter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(HerleProject src/Herle_Project.cpp)
target_link_libraries(HerleProject ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(HerleProjectPointCloud src/Herle_Project_PointCloud.cpp)
target_link_libraries(HerleProjectPointCloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(stlWriter src/stlWriter.cpp)
target_link_libraries(stlWriter ${catkin_LIBRARIES} ${VTK_LIBRARIES})

add_executable(EuclideanClusterCategorizer src/EuclideanClusterCategorizer.cpp)
target_link_libraries(EuclideanClusterCategorizer stlWriter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

#add_executable(SendStampedPose src/SendStampedPose.cpp)
#target_link_libraries(SendStampedPose ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(ThreadExample src/ThreadExample.cpp)
#target_link_libraries(ThreadExample ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(stl_loader src/stl_loader.cpp)
add_executable(stl_to_mesh src/stl_to_mesh.cpp)
target_link_libraries(stl_to_mesh stl_loader ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${Boost_LIBRARIES})
