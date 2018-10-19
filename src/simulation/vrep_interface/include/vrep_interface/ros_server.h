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

class ROSServer
{
  public:
    ROSServer(){};
    ~ROSServer();
    bool initialize();
    void addStatusBarMessageCallback(const std_msgs::String::ConstPtr &msg);
    void spinOnce();
    void simulationAboutToStart();
    void simulationEnded();

    static void info(const std::string &message);
    static void warn(const std::string &message);
    static void error(const std::string &message);

  private:
    ros::NodeHandle *nh;
    ros::ServiceServer spawn_robot_server;
    ros::ServiceServer spawn_robot_random_server;
    ros::ServiceServer shutdown_vrep_server;
    ros::Publisher *clock_publisher;
    ros::Subscriber add_status_bar_message_subscriber;
    tf::TransformBroadcaster *tf_broadcaster;

    VREPRobot *robot;

    bool spawnRobotService(vrep_msgs::SpawnRobot::Request &req, vrep_msgs::SpawnRobot::Response &res);
    bool spawnRobotRandomService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool shutdownService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  };

}

#endif
