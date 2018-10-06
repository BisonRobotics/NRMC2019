#include <vrep_interface/vrep_server.h>


using namespace vrep_interface;


/*******************************************************************************
 * Global variables
 ******************************************************************************/
ros::NodeHandle *VREPServer::nh = NULL;
ros::ServiceServer VREPServer::spawn_robot_server;
ros::ServiceServer VREPServer::spawn_robot_random_server;
ros::ServiceServer VREPServer::shutdown_vrep_server;
tf::TransformBroadcaster* VREPServer::tf_broadcaster = NULL;
ros::Subscriber VREPServer::add_status_bar_message_subscriber;
VREPRobot VREPServer::robot;


/*******************************************************************************
 * VREP template methods
 ******************************************************************************/
bool VREPServer::initialize()
{
  int argc = 0; char **argv = NULL;
  ros::init(argc, argv, "vrep");
  if (!ros::master::check()) return false;
  nh = new ros::NodeHandle("~");

  std::string description_path = ros::package::getPath("description");
  if (description_path.empty())
  {
    ROS_ERROR("[vrep_interface]: Unable to find the description package path, have you sourced your workspace?");
  }

  spawn_robot_server = nh->advertiseService("spawn_robot", VREPServer::spawnRobotService);
  spawn_robot_random_server = nh->advertiseService("spawn_robot_random", VREPServer::spawnRobotRandomService);
  shutdown_vrep_server = nh->advertiseService("shutdown", VREPServer::shutdownService);
  add_status_bar_message_subscriber = nh->subscribe("addStatusbarMessage", 1,
                                                      &VREPServer::addStatusBarMessageCallback);
  VREPServer::tf_broadcaster = new tf::TransformBroadcaster();

  simLoadScene((description_path + "/vrep_models/arena.ttt").c_str());

  robot.initialize(description_path + "/vrep_models/robot.ttm");
  robot.spawnRobot();

  return true;
}

void VREPServer::shutDown()
{
  add_status_bar_message_subscriber.shutdown();
  spawn_robot_server.shutdown();
  spawn_robot_random_server.shutdown();
  shutdown_vrep_server.shutdown();
  robot.shutdown();
  ros::shutdown();
}

void VREPServer::instancePass() // Simulation not running
{
  if ((simGetSimulationState() & sim_simulation_advancing) == 0)
  {
    ros::spinOnce();
  }
}

void VREPServer::mainScriptAboutToBeCalled() // Simulation running
{
  spinOnce();
}

void VREPServer::simulationAboutToStart() {}
void VREPServer::simulationEnded() {}

void VREPServer::spinOnce()
{
  // Disable error reporting (it is enabled in the service processing part, but
  // we don't want error reporting for publishers/subscribers)
  int errorModeSaved;
  simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
  simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);

  // Process all requested services and topic subscriptions
  ros::spinOnce();
  robot.spinOnce();
  tf::Transform robot_position;
  robot.getPosition(&robot_position);
  tf_broadcaster->sendTransform(tf::StampedTransform(robot_position, ros::Time::now(), "map", "base_link"));

  // Restore previous error report mode:
  simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);
}

/*******************************************************************************
 * Custom services
 ******************************************************************************/
bool VREPServer::spawnRobotRandomService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  try
  {
    robot.spawnRobot();
    return true;
  }
  catch (std::runtime_error &e)
  {
    return false;
  }
}

bool VREPServer::spawnRobotService(vrep_msgs::SpawnRobot::Request &req,
                                   vrep_msgs::SpawnRobot::Response &res)
{
  robot.spawnRobot(req.x, req.y, req.omega);
}


bool VREPServer::shutdownService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  simAddStatusbarMessage("[Service shutdownService] Trying to shutdown");
  res.success = 1;
  res.message = "Trying to shutdown...";
  simQuitSimulator(1);
  return true;
}

void VREPServer::addStatusBarMessageCallback(const std_msgs::String::ConstPtr &msg)
{
  simAddStatusbarMessage(msg->data.c_str());
}





