#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dig_control_2/dig_controller/dig_controller.h>

using namespace dig_control_2;

float central_duty, backhoe_duty, vibrator_duty, bucket_duty, left_wheels, right_wheels;
bool drive_safety, dig_safety;


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
  float ls = joy->axes[1];        // Left wheels
  float rs = joy->axes[3];        // Right wheels
  bool up = joy->axes[5] >  0.5;  // Central drive up (Up on left pad)
  bool dp = joy->axes[5] < -0.5;  // Central drive down (Down on left pad)

  drive_safety = lb;
  dig_safety = rb;

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
      bucket_duty = -DigController::BucketDuty::normal;
    }
    else if (y)
    {
      bucket_duty = DigController::BucketDuty::normal;
    }

    // Update backhoe (Maintain state)
    if (a && b)
    {
      ROS_WARN("[teleop] Conflicting commands, a and b are both pressed, stopping motion");
      backhoe_duty = 0.0f;
    }
    else if (a)
    {
      backhoe_duty = -DigController::BackhoeDuty::slow;
    }
    else if (b)
    {
      backhoe_duty = DigController::BackhoeDuty::slow;
    }

    // Update central drive
    if (up && dp)
    {
      ROS_WARN("[teleop] Conflicting commands, up and dp are both pressed, stopping motion");
      central_duty = 0.0f;
    }
    else if (up)
    {
      central_duty = DigController::CentralDriveDuty::slow;
    }
    else if (dp)
    {
      central_duty = -DigController::CentralDriveDuty::slow;
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
      vibrator_duty = DigController::VibratorDuty::normal;
    }
  }
  else
  {
    central_duty = 0.0f;
    backhoe_duty = 0.0f;
    bucket_duty = 0.0f;
    vibrator_duty = 0.0f;
  }

  ROS_INFO("[teleop] Dv = %i,L = %4.2f, R = %4.2f",
           (int)drive_safety, left_wheels, right_wheels);
  ROS_INFO("[teleop] Dg = %i, C = %4.2f, Bh = %4.1f, Bt = %4.1f, V = %3.1f",
           (int)dig_safety, central_duty, backhoe_duty, bucket_duty, vibrator_duty);
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

  DigController dig_controller;
  dig_controller.setControlState(DigController::ControlState::manual);

  while (ros::ok())
  {
    dig_controller.update();
    ros::spinOnce();
    if (dig_safety)
    {
      dig_controller.setCentralDriveDuty(central_duty);
      dig_controller.setBackhoeDuty(backhoe_duty);
      dig_controller.setVibratorDuty(vibrator_duty);
      dig_controller.setBucketDuty(bucket_duty);
    }
    else
    {
      dig_controller.stop();
    }
    rate.sleep();
  }
}
