#ifndef VREP_INTERFACE_VREP_SERVER_H
#define VREP_INTERFACE_VREP_SERVER_H


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

#include <vrep_plugin/robot.h>
#include <vrep_msgs/SpawnRobot.h>
#include <vrep_plugin/interface.h>

namespace vrep_plugin
{

class Server
{
  public:
    explicit Server(Interface *sim_interface);
    ~Server();
    void spinOnce();
    void simulationAboutToStart();
    void simulationEnded();

  private:
    Interface *sim;

    bool sim_running;
    ros::NodeHandle *nh;
    ros::ServiceServer spawn_robot_server;
    ros::ServiceServer spawn_robot_random_server;
    ros::ServiceServer shutdown_server;
    ros::ServiceServer start_server;
    ros::ServiceServer pause_server;
    ros::ServiceServer stop_server;
    ros::Publisher *clock_publisher;
    ros::Publisher *pose_publisher;

    Robot *robot;
    unsigned int message_counter;

    bool spawnRobot(vrep_msgs::SpawnRobot::Request &req, vrep_msgs::SpawnRobot::Response &res);
    bool spawnRobotRandom(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool start(std_srvs::TriggerRequest &req, std_srvs::Trigger::Response &res);
    bool pause(std_srvs::TriggerRequest &req, std_srvs::Trigger::Response &res);
    bool stop(std_srvs::TriggerRequest &req, std_srvs::Trigger::Response &res);
    bool shutdown(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};

}

#endif //VREP_INTERFACE_VREP_SERVER_H
