//#include <frosty_state_machine.h>

#include <ros/ros.h>


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
    
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        ROS_DEBUG("**");
    }
}