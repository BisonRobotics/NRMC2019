#include "vrep_interface/vrep_server.h"

using namespace vrep_interface;

VREPServer::VREPServer()
{
  int argc = 0; char **argv = NULL;
  ros::init(argc, argv, "vrep");
  if (!ros::master::check())
  {
    simAddStatusbarMessage("[WARN]: Unable to start, ros master isn't running");
  }
  nh = new ros::NodeHandle("~");
  std::cout << "Starting server..." << std::endl;

  std::string description_path = ros::package::getPath("description");
  if (description_path.empty())
  {
    std::cout << "Unable to find the description package path, have you sourced your workspace?" << std::endl;
  }

  std::cout << "Initialized" << std::endl;
}

VREPServer::~VREPServer()
{
  nh->shutdown();
  ros::shutdown();
}
