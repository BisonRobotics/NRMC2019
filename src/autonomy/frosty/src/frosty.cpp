//#include <frosty_state_machine.h>

#include <ros/ros.h>
#include <frosty_state_machine/frosty_state_machine.h>


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
    
    FrostyStateMachine FSM(true, true, 10, 10);
    
    while (ros::ok())
    {
        FSM.update(.1);
        
        r.sleep();
        ros::spinOnce();
        ROS_DEBUG("**");
    }
}