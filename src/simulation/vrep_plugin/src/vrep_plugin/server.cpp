#include "vrep_plugin/server.h"
#include <rosgraph_msgs/Clock.h>
#include <vrep_plugin/server.h>

using namespace vrep_plugin;

Server::Server(Interface *sim_interface)
{
  sim = sim_interface;
  sim_running = false;

  int argc = 0; char **argv = NULL;
  ros::init(argc, argv, "vrep");
  if (!ros::master::check())
  {
    simAddStatusbarMessage("[WARN]: Unable to start, ros master isn't running");
    throw std::runtime_error("Unable to start, ros master isn't running");
  }
  nh = new ros::NodeHandle("~");
  sim->info("[Server]: Starting...");

  std::string description_path = ros::package::getPath("description");
  if (description_path.empty())
  {
    sim->error("Unable to find the description package path, have you sourced your workspace?");
    throw std::runtime_error("Unable to find the description package path, have you sourced your workspace?");
  }
  spawn_robot_server = nh->advertiseService("spawn_robot", &Server::spawnRobotService, this);
  spawn_robot_random_server = nh->advertiseService("spawn_robot_random", &Server::spawnRobotRandomService, this);
  shutdown_vrep_server = nh->advertiseService("shutdown", &Server::shutdownService, this);
  tf_broadcaster = new tf::TransformBroadcaster();
  clock_publisher = new ros::Publisher;
  (*clock_publisher) = nh->advertise<rosgraph_msgs::Clock>("/clock", 10, true);

  sim->loadScene(description_path + "/vrep_models/arena.ttt");

  robot = new Robot(sim);
  robot->initialize(description_path + "/vrep_models/robot.ttm");
  robot->spawnRobot();

  sim->info("[Server]: Ready");
}

Server::~Server()
{
  spawn_robot_server.shutdown();
  spawn_robot_random_server.shutdown();
  shutdown_vrep_server.shutdown();
  robot->shutdown();
  nh->shutdown();
  ros::shutdown();
}

void Server::spinOnce()
{
  sim->disableErrorReporting();

  try
  {
    if (sim_running)
    {
      rosgraph_msgs::Clock sim_clock = sim->getSimulationTime();
      clock_publisher->publish(sim_clock);

      robot->spinOnce();
      tf::Transform robot_position;
      robot->getPosition(&robot_position);
      tf_broadcaster->sendTransform(tf::StampedTransform(robot_position, ros::Time::now(), "map", "base_link"));
    }
    ros::spinOnce();
  }
  catch (const std::exception &e)
  {
    sim->error(e.what());
  }

  sim->resumeErrorReporting();
}

void Server::simulationAboutToStart()
{
  sim_running = true;
}

void Server::simulationEnded()
{
  sim_running = false;
}

bool Server::spawnRobotRandomService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  try
  {
    robot->spawnRobot();
    return true;
  }
  catch (const std::runtime_error &e)
  {
    sim->error(e.what());
    return false;
  }
}

bool Server::spawnRobotService(vrep_msgs::SpawnRobot::Request &req,
                                   vrep_msgs::SpawnRobot::Response &res)
{
  try
  {
    robot->spawnRobot(req.x, req.y, req.omega);
  }
  catch (const std::runtime_error &e)
  {
    sim->error(e.what());
    return false;
  }
}

bool Server::shutdownService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  sim->info("[shutdownService] Trying to shutdown");
  res.success = 1;
  res.message = "Trying to shutdown...";
  simQuitSimulator(1);
  return true;
}
