#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dig_control/dig_controller.h>

using namespace dig_control;

bool drive_safety, dig_safety, automatic_dig;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  bool x  = joy->buttons[0] == 1; // Dump
  bool y  = joy->buttons[3] == 1; // Finish dump
  bool a  = joy->buttons[1] == 1; // Dig
  bool b  = joy->buttons[2] == 1; // Finish dig
  bool rb = joy->buttons[5] == 1; // Dig safety

  dig_safety = rb;

  if (dig_safety)
  {

  }
  else
  {

  }
}

int main(int argc, char **argv)
{
  drive_safety = false;
  dig_safety = false;

  ros::init(argc, argv, "dig_teleop");
  ros::NodeHandle n;
  ros::Subscriber joy_sub = n.subscribe("joy", 2, callback);
  ros::Rate rate(50);

  DigController dig_controller;
  dig_controller.setControlState(ControlState::manual);

  while (ros::ok())
  {
    dig_controller.update();
    ros::spinOnce();
    ControlState dig_state = dig_controller.getControlState();
    if (dig_safety)
    {
      if (dig_state == ControlState::error)
      {
        ROS_ERROR("Dig controller is in an error state");
        dig_controller.stop();
      }
      else if (automatic_dig)
      {
        if (dig_controller.getControlState() != ControlState::dig)
        {
          ROS_INFO("Setting control mode to dig");
          dig_controller.setControlState(ControlState::dig);
        }
      }
      else
      {
        if (dig_controller.getControlState() != ControlState::manual)
        {
          dig_controller.setControlState(ControlState::manual);
        }
        dig_controller.setCentralDriveDuty(central_duty);
        dig_controller.setBackhoeDuty(backhoe_duty);
        dig_controller.setVibratorDuty(vibrator_duty);
        dig_controller.setBucketDuty(bucket_duty);
      }
    }
    else
    {
      dig_controller.setControlState(ControlState::manual);
      dig_controller.stop();
    }
    if (drive_safety)
    {
      //ROS_INFO("| L | %f | R | %f |", left_wheels, right_wheels);
      teleop.update(left_wheels, right_wheels);
    }
    else
    {
      teleop.update(0.0f, 0.0f);
    }
    /*ROS_INFO("| P | %5i | DS | %15s | CD | %20s | BH | %9s |",
        dig_controller.getCentralDrivePosition(),
        dig_controller.getDigStateString().c_str(),
        dig_controller.getCentralDriveStateString().c_str(),
        dig_controller.getBackhoeStateString().c_str());*/
    rate.sleep();
  }
}

