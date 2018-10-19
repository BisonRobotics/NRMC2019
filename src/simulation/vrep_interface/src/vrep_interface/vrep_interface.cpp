#include <vrep_interface/vrep_interface.h>


using namespace vrep_interface;

/*******************************************************************************
 * Global variables
 ******************************************************************************/
//ros::NodeHandle *VREPInterface::nh = NULL;
VREPServer *VREPInterface::server = NULL;
/*ros::ServiceServer VREPInterface::spawn_robot_server;
ros::ServiceServer VREPInterface::spawn_robot_random_server;
ros::ServiceServer VREPInterface::shutdown_vrep_server;
ros::Subscriber VREPInterface::add_status_bar_message_subscriber;
ros::Publisher *VREPInterface::clock_publisher = NULL;
tf::TransformBroadcaster* VREPInterface::tf_broadcaster = NULL;
VREPRobot *VREPInterface::robot = NULL;*/


/*******************************************************************************
 * VREP template methods
 ******************************************************************************/
bool VREPInterface::initialize()
{
  try
  {
    server = new VREPServer;
  }
  catch (std::exception &e)
  {
    return false;
  }
  return true;



  /*spawn_robot_server = nh->advertiseService("spawn_robot", VREPInterface::spawnRobotService);
  spawn_robot_random_server = nh->advertiseService("spawn_robot_random", VREPInterface::spawnRobotRandomService);
  shutdown_vrep_server = nh->advertiseService("shutdown", VREPInterface::shutdownService);
  add_status_bar_message_subscriber = nh->subscribe("addStatusbarMessage", 1,
                                                      &VREPInterface::addStatusBarMessageCallback);
  tf_broadcaster = new tf::TransformBroadcaster();
  clock_publisher = new ros::Publisher;
  (*clock_publisher) = nh->advertise<rosgraph_msgs::Clock>("/clock", 10, true);

  simInt status = simLoadScene((description_path + "/vrep_models/arena.ttt").c_str());
  if (status == -1)
  {
    error("Unable to load scene");
  }

  robot = new VREPRobot;
  try
  {
    robot->initialize(description_path + "/vrep_models/robot.ttm");
    robot->spawnRobot();
  }
  catch (const std::exception &e)
  {
    error(e.what());
    return false;
  }
  info("Server started");*/

}

void VREPInterface::shutDown()
{
  /*add_status_bar_message_subscriber.shutdown();
  spawn_robot_server.shutdown();
  spawn_robot_random_server.shutdown();
  shutdown_vrep_server.shutdown();
  robot->shutdown();*/
  //nh->shutdown();
  //ros::shutdown();
  delete server;
}

void VREPInterface::instancePass() // Simulation not running
{
  if ((simGetSimulationState() & sim_simulation_advancing) == 0)
  {
    //ros::spinOnce();
  }
}

void VREPInterface::mainScriptAboutToBeCalled() // Simulation running
{
  //spinOnce();
}

void VREPInterface::simulationAboutToStart()
{
  //info("Starting simulation");
}

void VREPInterface::simulationEnded()
{
  //info("Simulation ended");
}

/*void VREPInterface::spinOnce()
{*/
  // Disable error reporting (it is enabled in the service processing part, but
  // we don't want error reporting for publishers/subscribers)
  /*int errorModeSaved;
  simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
  simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);

  // Process all requested services and topic subscriptions
  try
  {
    ros::Time sim_time(simGetSimulationTime());
    rosgraph_msgs::Clock sim_clock;
    sim_clock.clock.nsec = sim_time.nsec;
    sim_clock.clock.sec = sim_time.sec;
    clock_publisher->publish(sim_clock);

    ros::spinOnce();
    robot->spinOnce();

    tf::Transform robot_position;
    robot->getPosition(&robot_position);
    tf_broadcaster->sendTransform(tf::StampedTransform(robot_position, ros::Time::now(), "map", "base_link"));
  }
  catch (const std::exception &e)
  {
    error(e.what());
  }

  // Restore previous error report mode:
  simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);*/
//}

/*******************************************************************************
 * Custom services
 ******************************************************************************/


void VREPInterface::info(const std::string &message)
{
  simAddStatusbarMessage(("[INFO]: " + message).c_str());
  std::cout << message.c_str() << std::endl;
}

void VREPInterface::warn(const std::string &message)
{
  simAddStatusbarMessage(("[WARN]: " + message).c_str());
  std::cout << message.c_str() << std::endl;
}

void VREPInterface::error(const std::string &message)
{
  simAddStatusbarMessage(("[ERROR]: " + message).c_str());
  std::cout << message.c_str() << std::endl;
}






