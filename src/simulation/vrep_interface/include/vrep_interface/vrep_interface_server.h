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

#include <vrep_library/v_repLib.h>

#include <vrep_robot/vrep_robot.h>
#include <vrep_msgs/SpawnRobot.h>

namespace vrep_interface
{

class vrep_interface_server
{
public:
  static bool initialize();
  static void shutDown();

  static void instancePass();
  static void mainScriptAboutToBeCalled();

  static void simulationAboutToStart();
  static void simulationEnded();

private:
  vrep_interface_server(){};

  static ros::NodeHandle *nh;

  static void spinOnce();

  // Services:
  static bool spawnRobotService(vrep_msgs::SpawnRobot::Request &req,
                                vrep_msgs::SpawnRobot::Response &res);
  static bool spawnRobotRandomService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  static bool shutdownService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  static ros::ServiceServer spawn_robot_server;
  static ros::ServiceServer spawn_robot_random_server;
  static ros::ServiceServer shutdown_vrep_server;

  // TF Broadcaster
  static tf::TransformBroadcaster* tf_broadcaster;

  // Subscribers:
  static void addStatusBarMessageCallback(const std_msgs::String::ConstPtr &msg);

  static ros::Subscriber add_status_bar_message_subscriber;

};

}

#endif
