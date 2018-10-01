#ifndef ROS_SERVER_H
#define ROS_SERVER_H


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>

#include <vrep_interface/VREPInterfaceConfig.h>
#include <vrep_lib/v_repLib.h>

#include <vrep_robot/vrep_robot.h>
#include <obstacle_field/obstacle_field.h>
#include <vrep_msg_and_srv/spawnRobot.h>
#include <camera/camera.h>


namespace vrep_interface
{

class ros_server
{
public:
  static bool initialize();
  static void shutDown();

  static void instancePass();
  static void mainScriptAboutToBeCalled();

  static void simulationAboutToStart();
  static void simulationEnded();

private:
  ros_server(){};

  static ros::NodeHandle *node;

  static void spinOnce();

  // Services:
  static bool spawnRobotService(vrep_msg_and_srv::spawnRobot::Request &req,
                                vrep_msg_and_srv::spawnRobot::Response &res);
  static bool spawnRobotRandomService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  static bool shutdownService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  static bool generateRandomObstacleField(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  static ros::ServiceServer spawn_robot_server;
  static ros::ServiceServer spawn_robot_random_server;
  static ros::ServiceServer shutdown_vrep_server;
  static ros::ServiceServer generate_obstacle_field_server;

  // Publishers:
  static void streamAllData();
  static ros::Publisher object_count_publisher;
  //static ros::Publisher laser_scan_publisher;
  static ros::Publisher map_publisher;

  // TF Broadcaster
  static tf::TransformBroadcaster* tf_broadcaster;

  // Subscribers:
  static void addStatusBarMessageCallback(const std_msgs::String::ConstPtr &msg);
  static void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  static ros::Subscriber add_status_bar_message_subscriber;
  static ros::Subscriber cmd_vel_subscriber;

  // Dynamic Reconfigure
  static dynamic_reconfigure::Server<vrep_interface::VREPInterfaceConfig> *reconfig_server;
  static void config_callback(vrep_interface::VREPInterfaceConfig &config, uint32_t level);
};

}

#endif
