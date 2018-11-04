#include "vrep_plugin/server.h"
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/PoseStamped.h>
#include <vrep_plugin/server.h>

using namespace vrep_plugin;

using vrep_msgs::SpawnRobot;
using std_srvs::Trigger;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;

Server::Server(Interface *sim_interface)
{
  sim = sim_interface;
  sim_running = false;
  message_counter = 0;

  int argc = 0; char **argv = NULL;
  ros::init(argc, argv, "vrep");
  if (!ros::master::check())
  {
    simAddStatusbarMessage("[WARN]: Unable to start, ros master isn't running");
    std::cout << "[WARN]: Unable to start, ros master isn't running" << std::endl;
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

  spawn_robot_server = nh->advertiseService("spawn_robot", &Server::spawnRobot, this);
  spawn_robot_random_server = nh->advertiseService("spawn_robot_random", &Server::spawnRobotRandom, this);
  start_server = nh->advertiseService("start", &Server::start, this);
  pause_server = nh->advertiseService("pause", &Server::pause, this);
  stop_server = nh->advertiseService("stop", &Server::stop, this);
  shutdown_server = nh->advertiseService("shutdown", &Server::shutdown, this);

  clock_publisher = new ros::Publisher;
  pose_publisher = new ros::Publisher;
  (*clock_publisher) = nh->advertise<rosgraph_msgs::Clock>("/clock", 10, true);
  (*pose_publisher) = nh->advertise<PoseStamped>("pose", 10, true);

  sim->loadScene(description_path + "/vrep_models/arena.ttt");

  robot = new Robot(sim);
  robot->initialize(description_path + "/vrep_models/robot.ttm");
  robot->spawnRobot();

  bool real_time;
  nh->param<bool>("real_time", real_time, true);
  if (real_time)
  {
    sim->info("[Server]: Real time simulation enabled");
  }
  else
  {
    sim->info("Real time simulation disabled");
  }
  sim->setParameter(sim_boolparam_realtime_simulation, real_time);

  sim->info("[Server]: Ready");
}

Server::~Server()
{
  spawn_robot_server.shutdown();
  spawn_robot_random_server.shutdown();
  shutdown_server.shutdown();
  robot->shutdown();
  clock_publisher->shutdown();
  pose_publisher->shutdown();
  nh->shutdown();
  ros::shutdown();
  delete robot;
  delete clock_publisher;
  delete pose_publisher;
}

void Server::spinOnce()
{
  sim->disableErrorReporting();

  try
  {
    if (sim_running)
    {
      message_counter++;

      rosgraph_msgs::Clock sim_clock;
      sim_clock.clock = sim->getSimulationTime();
      clock_publisher->publish(sim_clock);

      robot->spinOnce();

      Pose pose;
      robot->getPose(&pose);
      PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.seq = message_counter;
      pose_stamped.pose = pose;
      pose_publisher->publish(pose_stamped);
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
  robot->reset();
}

void Server::simulationEnded()
{
  sim_running = false;
}

bool Server::spawnRobotRandom(Trigger::Request &req, Trigger::Response &res)
{
  try
  {
    robot->spawnRobot();
    res.success = 1;
    res.message = "success";
    return true;
  }
  catch (const std::runtime_error &e)
  {
    sim->error(e.what());
    res.message = e.what();
    return false;
  }
}

bool Server::spawnRobot(SpawnRobot::Request &req, SpawnRobot::Response &res)
{
  try
  {
    robot->spawnRobot(req.x, req.y, req.omega);
    res.success = 1;
    return true;
  }
  catch (const std::runtime_error &e)
  {

    sim->error(e.what());
    return false;
  }
}

bool Server::start(Trigger::Request &req, Trigger::Response &res)
{
  try
  {
    sim->startSimulation();
    res.success = 1;
    res.message = "success";
    return true;
  }
  catch (const std::runtime_error &e)
  {
    sim->error(e.what());
    res.success = 0;
    res.message = e.what();
    return false;
  }
}

bool Server::pause(Trigger::Request &req, Trigger::Response &res)
{
  try
  {
    sim->pauseSimulation();
    res.success = 1;
    res.message = "success";
    return true;
  }
  catch (const std::runtime_error &e)
  {
    sim->error(e.what());
    res.success = 0;
    res.message = e.what();
    return false;
  }
}

bool Server::stop(Trigger::Request &req, Trigger::Response &res)
{
  try
  {
    sim->stopSimulation();
    res.success = 1;
    res.message = "success";
    return true;
  }
  catch (const std::runtime_error &e)
  {
    sim->error(e.what());
    res.success = 0;
    res.message = e.what();
    return false;
  }
}

bool Server::shutdown(Trigger::Request &req, Trigger::Response &res)
{
  sim->info("[Server]: Shutting down...");
  sim->shutdown();
  res.success = 1;
  res.message = "Shutting down...";
  return true;
}
