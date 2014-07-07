#include "ros/ros.h"
#include "gazebo_pkg/GetCoordFromClient.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_coordinates_client");
  if (argc != 4)
  {
    ROS_INFO("usage: add_coordinates_client X Y Z");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_pkg::GetCoordFromClient>("get_coordinates");

//  gazebo_pkg::GetCoordFromClient srv;
//  srv.request.x = atoll(argv[1]);
//  srv.request.y = atoll(argv[2]);
//  srv.request.z = atoll(argv[3]);
  if (client.call(srv))
  {
    ROS_INFO("Am preluat coordonatele!");
  }
  else
  {
    ROS_ERROR("N-am reusit sa preiau coordonatele!");
    return 1;
  }

  return 0;
}
