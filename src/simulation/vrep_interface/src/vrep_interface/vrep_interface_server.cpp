#include <vrep_interface/vrep_interface_server.h>


using namespace vrep_interface;

#define TRUE = 1;
#define FALSE = 0;


/*
 * Global variables
 */
ros::NodeHandle *vrep_interface_server::nh = NULL;

// Services:
ros::ServiceServer vrep_interface_server::spawn_robot_server;
ros::ServiceServer vrep_interface_server::spawn_robot_random_server;
ros::ServiceServer vrep_interface_server::shutdown_vrep_server;

// TF Broadcaster
tf::TransformBroadcaster* vrep_interface_server::tf_broadcaster = NULL;

// Subscribers:
ros::Subscriber vrep_interface_server::add_status_bar_message_subscriber;

// Robot
VREPRobot robot;


/*
 * VREP template methods
 */
bool vrep_interface_server::initialize()
{
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "vrep_interface");
  if (!ros::master::check()) return false;
  nh = new ros::NodeHandle("~");

  std::string description_path = ros::package::getPath("description");
  if (description_path.empty())
  {
    ROS_ERROR("[vrep_interface]: Unable to find the description package path, have you sourced your workspace?");
  }

  spawn_robot_server = nh->advertiseService("spawn_robot", vrep_interface_server::spawnRobotService);
  spawn_robot_random_server = nh->advertiseService("spawn_robot_random", vrep_interface_server::spawnRobotRandomService);
  shutdown_vrep_server = nh->advertiseService("shutdown_vrep", vrep_interface_server::shutdownService);
  add_status_bar_message_subscriber = nh->subscribe("addStatusbarMessage", 1,
                                                      &vrep_interface_server::addStatusBarMessageCallback);
  vrep_interface_server::tf_broadcaster = new tf::TransformBroadcaster();

  robot.setModelFile(description_path + "/vrep_models/robot.ttm");
  simLoadScene((description_path + "/vrep_models/arena.ttt").c_str());
  robot.spawnRobot();

  return true;
}


void vrep_interface_server::shutDown()
{
  add_status_bar_message_subscriber.shutdown();
  spawn_robot_server.shutdown();
  spawn_robot_random_server.shutdown();
  shutdown_vrep_server.shutdown();
  ros::shutdown();
}

// When simulation is not running, we "spinOnce" here:
void vrep_interface_server::instancePass()
{
  if ((simGetSimulationState() & sim_simulation_advancing) == 0)
  {
    ros::spinOnce();
  }
}

// When simulation is running, we "spinOnce" here:
void vrep_interface_server::mainScriptAboutToBeCalled()
{
  spinOnce();
}

void vrep_interface_server::simulationAboutToStart() {}
void vrep_interface_server::simulationEnded() {}

void vrep_interface_server::spinOnce()
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

// Subscribers:
void vrep_interface_server::addStatusBarMessageCallback(const std_msgs::String::ConstPtr &msg)
{
  simAddStatusbarMessage(msg->data.c_str());
}

/*
 * Custom services
 */
bool vrep_interface_server::spawnRobotRandomService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
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

bool vrep_interface_server::spawnRobotService(vrep_msgs::SpawnRobot::Request &req,
                                   vrep_msgs::SpawnRobot::Response &res)
{
  robot.spawnRobot(req.xPos, req.yPos, req.rotFromXAxis);
}


bool vrep_interface_server::shutdownService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  simAddStatusbarMessage("[Service shutdownService] Trying to shutdown");
  res.success = 1;
  res.message = "Trying to shutdown...";
  simQuitSimulator(1);
  return true;
}





