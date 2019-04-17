#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <teleop_interface/teleop_interface.h>
#include <dig_control/dig_controller/dig_controller.h>

using namespace dig_control;

float central_duty, backhoe_duty, vibrator_duty, bucket_duty, left_wheels, right_wheels;
bool drive_safety, dig_safety, automatic_dig;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  bool x  = joy->buttons[0] == 1; // Bucket down
  bool a  = joy->buttons[1] == 1; // Linear actuator in
  bool b  = joy->buttons[2] == 1; // Linear actuator out
  bool y  = joy->buttons[3] == 1; // Bucket up
  bool lb = joy->buttons[4] == 1; // Drive safety
  bool rb = joy->buttons[5] == 1; // Dig safety
  bool lt = joy->buttons[6] == 1; // Vibrator off
  bool rt = joy->buttons[7] == 1; // Vibrator on
  bool st = joy->buttons[9] == 1; // Start automatic dig
  float ls = joy->axes[1];        // Left wheels
  float rs = joy->axes[3];        // Right wheels
  bool up = joy->axes[5] >  0.5;  // Central drive up (Up on left pad)
  bool dp = joy->axes[5] < -0.5;  // Central drive down (Down on left pad)

  drive_safety = lb;
  dig_safety = rb;
  automatic_dig = (st || automatic_dig) && dig_safety; // Maintain automatic digging until safety is released

  if (drive_safety)
  {
    left_wheels  = 0.25f * ls;
    right_wheels = 0.25f * rs;
  }
  else
  {
    left_wheels = 0.0f;
    right_wheels = 0.0f;
  }

  if (dig_safety)
  {
    // Update bucket (Maintain state)
    if (x && y)
    {
      ROS_WARN("[teleop] Conflicting commands, x and y are both pressed, stopping motion");
      bucket_duty = 0.0;
    }
    else if (x)
    {
      bucket_duty = -DigController::BucketDuty::fast;
    }
    else if (y)
    {
      bucket_duty = DigController::BucketDuty::fast;
    }

    // Update backhoe (Maintain state)
    if (a && b)
    {
      ROS_WARN("[teleop] Conflicting commands, a and b are both pressed, stopping motion");
      backhoe_duty = 0.0f;
    }
    else if (a)
    {
      backhoe_duty = -DigController::BackhoeDuty::fast;
    }
    else if (b)
    {
      backhoe_duty = DigController::BackhoeDuty::normal;
    }

    // Update central drive
    if (up && dp)
    {
      ROS_WARN("[teleop] Conflicting commands, up and dp are both pressed, stopping motion");
      central_duty = 0.0f;
    }
    else if (up)
    {
      central_duty = DigController::CentralDriveDuty::fast;
    }
    else if (dp)
    {
      central_duty = -DigController::CentralDriveDuty::normal;
    }
    else
    {
      central_duty = 0.0f;
    }

    // Update vibrator (Maintain state)
    if (lt && rt)
    {
      ROS_WARN("[teleop] Conflicting commands, lt and rt are both pressed, stopping motion");
      vibrator_duty = 0.0f;
    }
    else if (lt)
    {
      vibrator_duty = 0.0f;
    }
    else if (rt)
    {
      vibrator_duty = 0.75;
    }
  }
  else
  {
    central_duty = 0.0f;
    backhoe_duty = 0.0f;
    bucket_duty = 0.0f;
    vibrator_duty = 0.0f;
  }

  //ROS_INFO("[teleop] Dv = %i,L = %4.2f, R = %4.2f",
  //         (int)drive_safety, left_wheels, right_wheels);
  //ROS_INFO("[teleop] Dg = %i, C = %4.2f, Bh = %4.1f, Bt = %4.1f, V = %3.1f",
  //         (int)dig_safety, central_duty, backhoe_duty, bucket_duty, vibrator_duty);
}

int main(int argc, char **argv)
{
  drive_safety = false;
  dig_safety = false;
  central_duty = 0.0f;
  backhoe_duty = 0.0f;
  bucket_duty = 0.0f;
  vibrator_duty = 0.0f;
  left_wheels = 0.0f;
  right_wheels = 0.0f;

  ros::init(argc, argv, "dig_teleop");
  ros::NodeHandle n;
  ros::Subscriber joy_sub = n.subscribe("joy", 2, callback);
  ros::Rate rate(50);

  TeleopInterface teleop(TeleopInterface::duty, 0.95f);
  DigController dig_controller;
  dig_controller.setControlState(DigController::ControlState::manual);

  while (ros::ok())
  {
    dig_controller.update();
    ros::spinOnce();
    DigController::ControlState dig_state = dig_controller.getControlState();
    if (dig_safety)
    {
      if (dig_state == DigController::ControlState::error)
      {
        ROS_ERROR("Dig controller is in an error state");
        dig_controller.stop();
      }
      else if (automatic_dig)
      {
        if (dig_controller.getControlState() != DigController::ControlState::dig)
        {
          ROS_INFO("Setting control mode to dig");
          dig_controller.setControlState(DigController::ControlState::dig);
        }
      }
      else
      {
        if (dig_controller.getControlState() != DigController::ControlState::manual)
        {
          dig_controller.setControlState(DigController::ControlState::manual);
        }
        dig_controller.setCentralDriveDuty(central_duty);
        dig_controller.setBackhoeDuty(backhoe_duty);
        dig_controller.setVibratorDuty(vibrator_duty);
        dig_controller.setBucketDuty(bucket_duty);
      }
    }
    else
    {
      dig_controller.setControlState(DigController::ControlState::manual);
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

