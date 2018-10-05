#include <ros_interface_server/ros_server.h>


using namespace vrep_interface;

#define TRUE = 1;
#define FALSE = 0;


/*
 * Global variables
 */
ros::NodeHandle *ros_server::nh = NULL;

// Services:
ros::ServiceServer ros_server::spawn_robot_server;
ros::ServiceServer ros_server::spawn_robot_random_server;
ros::ServiceServer ros_server::shutdown_vrep_server;

// TF Broadcaster
tf::TransformBroadcaster* ros_server::tf_broadcaster = NULL;

// Subscribers:
ros::Subscriber ros_server::add_status_bar_message_subscriber;

// Obstacle field
ObstacleField obstacle_field;

// Robot
VREPRobot robot;


/*
 * VREP template methods
 */
bool ros_server::initialize()
{
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "vrep_interface");

  if (!ros::master::check())
  {
    return (false);
  }

  nh = new ros::NodeHandle("~");

  std::string description_path = ros::package::getPath("description");
  if (description_path == "")
  {
    ROS_ERROR("[vrep_interface]: Unable to find the description package path, have you sourced your workspace?");
  }

  // Enable the services:
  spawn_robot_server = nh->advertiseService("spawn_robot", ros_server::spawnRobotService);
  spawn_robot_random_server = nh->advertiseService("spawn_robot_random", ros_server::spawnRobotRandomService);
  shutdown_vrep_server = nh->advertiseService("shutdown_vrep", ros_server::shutdownService);

  // Enable the subscribers:
  add_status_bar_message_subscriber = nh->subscribe("addStatusbarMessage", 1,
                                                      &ros_server::addStatusBarMessageCallback);

  // TF Broadcaster
  ros_server::tf_broadcaster = new tf::TransformBroadcaster();

  // Initialize robot
  robot.setModelFile(description_path + "/vrep_models/robot.ttm");

  // Load scene
  simLoadScene((description_path + "/vrep_models/arena.ttt").c_str());

  robot.spawnRobot();

  return true;
}


void ros_server::shutDown()
{
  // Disable the subscribers:
  add_status_bar_message_subscriber.shutdown();

  // Disable the services:
  spawn_robot_server.shutdown();
  spawn_robot_random_server.shutdown();
  shutdown_vrep_server.shutdown();

  // Shut down:
  ros::shutdown();
}

// When simulation is not running, we "spinOnce" here:
void ros_server::instancePass()
{
  if ((simGetSimulationState() & sim_simulation_advancing) == 0)
  {
    ros::spinOnce();
  }
}

// When simulation is running, we "spinOnce" here:
void ros_server::mainScriptAboutToBeCalled()
{
  spinOnce();
}

void ros_server::simulationAboutToStart() {}
void ros_server::simulationEnded() {}

void ros_server::spinOnce()
{
  // Disable error reporting (it is enabled in the service processing part, but
  // we don't want error reporting for publishers/subscribers)
  int errorModeSaved;
  simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
  simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);

  // Process all requested services and topic subscriptions
  ros::spinOnce();

  // Restore previous error report mode:
  simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);

  // Update wheel velocities
  robot.spinOnce();

  // Broadcast position
  tf::Transform robot_position;
  robot.getPosition(&robot_position);
  tf_broadcaster->sendTransform(tf::StampedTransform(robot_position, ros::Time::now(), "map", "base_link"));
}

// Subscribers:
void ros_server::addStatusBarMessageCallback(const std_msgs::String::ConstPtr &msg)
{
  simAddStatusbarMessage(msg->data.c_str());
}

/*
 * Custom services
 */
bool ros_server::spawnRobotRandomService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = (uint8_t)robot.spawnRobot();
  return res.success;
}

bool ros_server::spawnRobotService(vrep_msg_and_srv::spawnRobot::Request &req,
                                   vrep_msg_and_srv::spawnRobot::Response &res)
{
  simFloat position[2] = {req.xPos, req.yPos};
  if(!robot.spawnRobot(position, req.rotFromXAxis)){
    res.success = 0;
    res.response = "Failed to load model";
    simAddStatusbarMessage(("[Service spawnRobot] " + res.response).c_str());
    return false;
  }
  res.success = 1;
  res.response = "Successfully loaded model";
  simAddStatusbarMessage(("[Service spawnRobot] " + res.response).c_str());
  return true;
}


bool ros_server::shutdownService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  simAddStatusbarMessage("[Service shutdownService] Trying to shutdown");
  res.success = 1;
  res.message = "Trying to shutdown...";
  simQuitSimulator(1);
  return true;
}





