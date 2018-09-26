#include <ros_interface_server/ros_interface_server.h>


using namespace vrep_interface;


/*
 * Global variables
 */
ros::NodeHandle *ros_server::node = NULL;

// Services:
ros::ServiceServer ros_server::spawn_robot_server;
ros::ServiceServer ros_server::spawn_robot_random_server;
ros::ServiceServer ros_server::shutdown_vrep_server;
ros::ServiceServer ros_server::generate_obstacle_field_server;

// Publishers:
ros::Publisher ros_server::object_count_publisher;
//ros::Publisher ros_server::laser_scan_publisher;
ros::Publisher ros_server::map_publisher;

// TF Broadcaster
tf::TransformBroadcaster* ros_server::tf_broadcaster = NULL;

// Subscribers:
ros::Subscriber ros_server::add_status_bar_message_subscriber;
ros::Subscriber ros_server::cmd_vel_subscriber;

// Dynamic Reconfigure
dynamic_reconfigure::Server<vrep_interface::VREPInterfaceConfig> *ros_server::reconfig_server;

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
  ros::init(argc, argv, "vrep");

  if (!ros::master::check())
  {
    return (false);
  }

  node = new ros::NodeHandle("~");

  std::string description_path = ros::package::getPath("description");
  if (description_path == "")
  {
    ROS_ERROR("Unable to find the description package path, have you sourced your workspace?");
  }

  // Setup reconfigure server
  reconfig_server = new dynamic_reconfigure::Server<vrep_interface::VREPInterfaceConfig>();
  dynamic_reconfigure::Server<vrep_interface::VREPInterfaceConfig>::CallbackType callback_type;
  callback_type = boost::bind(&ros_server::config_callback, _1, _2);
  reconfig_server->setCallback(callback_type);

  // Enable the services:
  spawn_robot_server = node->advertiseService("spawnRobot", ros_server::spawnRobotService);
  spawn_robot_random_server = node->advertiseService("spawn_robot_random", ros_server::spawnRobotRandomService);
  shutdown_vrep_server = node->advertiseService("shutdown_vrep", ros_server::shutdownService);
  generate_obstacle_field_server =
      node->advertiseService("generateRandomObstacleField", ros_server::generateRandomObstacleField);

  // Enable the publishers:
  object_count_publisher = node->advertise<std_msgs::Int32>("objectCount", 1);
  //laser_scan_publisher = node->advertise<sensor_msgs::LaserScan>("laser_scan_front", 1);
  map_publisher = node->advertise<nav_msgs::OccupancyGrid>("map", 1);

  // Enable the subscribers:
  add_status_bar_message_subscriber = node->subscribe("addStatusbarMessage", 1,
                                                      &ros_server::addStatusBarMessageCallback);
  cmd_vel_subscriber = node->subscribe("cmd_vel", 1, &ros_server::cmdVelCallback);

  // TF Broadcaster
  ros_server::tf_broadcaster = new tf::TransformBroadcaster();

  // Initialize robot
  robot.setModelFile(description_path + "/vrep_models/emydidae.ttm");

  // Load scene
  simLoadScene((description_path + "/scenes/arena.ttt").c_str());
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  generateRandomObstacleField(req, res);

  obstacle_field.initialize(description_path + "/vrep_models/");
  robot.spawnRobot();

  // For some reason VREP moves the camera during object placement, so we return it to it's original pose
  // after the field and robot have been placed.
  // TODO parametrize camera position and orientation
  Camera default_camera("DefaultCamera");
  simFloat position[3] = {-1.7684f, -5.6723f, 6.8113f};
  simFloat orientation[3] = {(simFloat)(-142.66 * M_PI / 180.0), (simFloat)(28.48f * M_PI / 180.0),
                             (simFloat)(-147.99f * M_PI / 180.0)};
  default_camera.set_position(position, orientation);

  return (true);
}


void ros_server::shutDown()
{
  // Disable the subscribers:
  add_status_bar_message_subscriber.shutdown();
  cmd_vel_subscriber.shutdown();

  // Disable the publishers:
  object_count_publisher.shutdown();
  //laser_scan_publisher.shutdown();
  map_publisher.shutdown();

  // Disable the services:
  spawn_robot_server.shutdown();
  spawn_robot_random_server.shutdown();
  shutdown_vrep_server.shutdown();
  generate_obstacle_field_server.shutdown();

  // Shut down:
  ros::shutdown();
}


// When simulation is not running, we "spinOnce" here:
void ros_server::instancePass()
{
  int simState = simGetSimulationState();
  if ((simState & sim_simulation_advancing) == 0) ros::spinOnce();
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
  simSetIntegerParameter(sim_intparam_error_report_mode,
                         sim_api_errormessage_ignore);

  // Handle all streaming (publishers)
  streamAllData();

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

  // Publish laser scan
  /*sensor_msgs::LaserScan scan;
  robot.getLaserScan(&scan);
  laser_scan_publisher.publish(scan);*/

  // Publish map
  nav_msgs::OccupancyGrid map;
  obstacle_field.get_occupancy_grid(&map);
  map_publisher.publish(map);
}


// Publishers:
void ros_server::streamAllData()
{
  // Take care of publishers here (i.e. have them publish their data):
  std_msgs::Int32 objCnt;
  int index = 0;
  int h = 0;
  while (h >= 0) {
    h = simGetObjects(index++, sim_handle_all);
  }
  objCnt.data = index - 1;
  object_count_publisher.publish(objCnt);
}


// Subscribers:
void ros_server::addStatusBarMessageCallback(const std_msgs::String::ConstPtr &msg)
{
  simAddStatusbarMessage(msg->data.c_str());
}


void ros_server::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  robot.setVelocity(msg.get()->linear.x, (simFloat) msg.get()->angular.z);
}


void ros_server::config_callback(vrep_interface::VREPInterfaceConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: interface: %s plugin: %s",
           config.wheel_control_interface.c_str(), config.wheel_control_plugin.c_str());
}


/*
 * Custom services for NRMC2018 simulation
 */
bool ros_server::spawnRobotRandomService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = (unsigned char) robot.spawnRobot();
  return res.success;
}


bool ros_server::spawnRobotService(vrep_msg_and_srv::spawnRobot::Request &req,
                                   vrep_msg_and_srv::spawnRobot::Response &res)
{
  // TODO Currently returns true to the ros server so that the service can
  // indicate error status, not sure if that the correct way to deal with errors
  simFloat position[2] = {req.xPos, req.yPos};
  if(!robot.spawnRobot(position, req.rotFromXAxis)){
    res.success = (unsigned char)false;
    res.response = "Failed to load model";
    simAddStatusbarMessage(("[Service spawnRobot] " + res.response).c_str());
    return false;
  }

  // Everything worked, indicate success
  res.success = (unsigned char)true;
  res.response = "Successfully loaded model";
  simAddStatusbarMessage(("[Service spawnRobot] " + res.response).c_str());
  return true;
}


bool ros_server::shutdownService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = (unsigned char)true;
  res.message = "Trying to shutdown...";
  simQuitSimulator(true);
  return true;
}


bool ros_server::generateRandomObstacleField(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  obstacle_field.generateFieldRandom();
  res.success = (simInt)true;
}





