#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <teleop_interface/teleop_interface.h>
#include <wheel_params/wheel_params.h>

float left_wheels, right_wheels;
bool drive_safety;

void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
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

  //ROS_INFO("[teleop] Dv = %i,L = %4.2f, R = %4.2f",
  //         (int)drive_safety, left_wheels, right_wheels);
  //ROS_INFO("[teleop] Dg = %i, C = %4.2f, Bh = %4.1f, Bt = %4.1f, V = %3.1f",
  //         (int)dig_safety, central_duty, backhoe_duty, bucket_duty, vibrator_duty);
}

int main(int argc, char **argv)
{
  drive_safety = false;
  left_wheels = 0.0f;
  right_wheels = 0.0f;

  ros::init(argc, argv, "teleop");
  ros::NodeHandle n;
  ros::Subscriber joy_sub = n.subscribe("joy", 2, joyCallback);
  ros::Rate rate(50);

  iVescAccess *fr, *fl, *br, *bl;
  fr = new VescAccess(front_right_param);
  fl = new VescAccess(front_left_param);
  br = new VescAccess(back_right_param);
  bl = new VescAccess(back_left_param);
  TeleopInterface teleop(TeleopInterface::Mode::velocity, 0.5, fl, fr, br, bl);

  while (ros::ok())
  {
    ros::spinOnce();
    if (drive_safety)
    {
      //ROS_INFO("| L | %f | R | %f |", left_wheels, right_wheels);
      teleop.update(left_wheels, right_wheels);
    }
    else
    {
      teleop.update(0.0f, 0.0f);
    }
    rate.sleep();
  }
}

