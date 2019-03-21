//#include <frosty_state_machine.h>

#include <ros/ros.h>
#include <frosty_state_machine/frosty_state_machine.h>

#include <std_srvs/Trigger.h>
#include <vrep_msgs/SpawnRobot.h>

int main (int argc, char **argv)
{
    ros::init (argc, argv, "frosty"); //Full Robotic Operations Simulation and Testing sYstem
    ros::NodeHandle nh;
    ros::Rate r (10);
    
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
    {
      ros::console::notifyLoggerLevelsChanged();
    }
    ROS_DEBUG("Debug info shown.");
    ROS_WARN("HELLO");
    
    bool simulating_driving = true;
    bool simulating_digging = true;
    double dig_time = 10;
    double dump_time = 10;
    
    if (simulating_driving)
    {
        //start vrep plugin via service call
        ros::ServiceClient start_client = nh.serviceClient<std_srvs::Trigger>("/vrep/start");
        std_srvs::Trigger strt;
        
        ros::ServiceClient spawn_client = nh.serviceClient<vrep_msgs::SpawnRobot>("/vrep/spawn_robot");
        vrep_msgs::SpawnRobot spwn;
        spwn.request.x     = 1;
        spwn.request.y     = 0;
        spwn.request.omega = 0;
        
        bool spawnd = false;
        bool startd = false;
        while (!(spawnd && startd))
        {
            r.sleep();
            if ( !startd)
            {
                bool res = start_client.call(strt);
                if (res)
                {
                   ROS_INFO("Called start vrep plugin.");
                   startd = true;
                }
                else
                {
                    ROS_WARN("CALL TO START VREP FAILED");
                }
            }
            else //started
            {
                if (!spawnd)
                {
                    bool res = spawn_client.call(spwn);
                    if (res)
                    {
                       ROS_INFO("Called spawn robot.");
                       spawnd = true;
                    }
                    else
                    {
                       ROS_WARN("CALL TO START VREP FAILED");
                    }
                }
            }
        }
    }
    
    FrostyStateMachine FSM(simulating_digging, simulating_driving, dig_time, dump_time);
    
    while (ros::ok())
    {
        FSM.update(.1);
        
        r.sleep();
        ros::spinOnce();
        ROS_DEBUG("**");
    }
}