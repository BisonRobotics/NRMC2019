#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <vrep_driver_access/vrep_driver_access.h>
#include <ros_driver_access/ros_driver_access.h>
#include <driver_access/params.h>
#include <teleop/teleop.h>

using namespace teleop;
using driver_access::Limits;
using driver_access::Mode;
using driver_access::DriverAccess;
using driver_access::VREPDriverAccess;
using driver_access::ID;
using driver_access::ROSDriverAccess;
using sensor_msgs::JointState;
using std::vector;

double left = 0.0;
double right = 0.0;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  if (joy->buttons[4])
  {
    left = joy->axes[1];
    right = joy->axes[4];
  }
  else
  {
    left = 0.0;
    right = 0.0;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "remote_control");
  ros::NodeHandle nh;
  ros::Subscriber joy_sub = nh.subscribe("joy", 1, callback);
  ros::Rate loop_rate(50);

  bool sim;
  nh.param("sim", sim, true);

  Limits limits(0, 0, 0, 1, 0, 1);
  DriverAccess *fl, *fr, *br, *bl;
  if (sim)
  {
    fl = new VREPDriverAccess(limits, ID::front_left_wheel,  Mode::velocity);
    fr = new VREPDriverAccess(limits, ID::front_right_wheel, Mode::velocity);
    br = new VREPDriverAccess(limits, ID::back_right_wheel,  Mode::velocity);
    bl = new VREPDriverAccess(limits, ID::back_left_wheel,   Mode::velocity);
  }
  else
  {
    // VESC implementation goes here
  }

  vector<DriverAccess*> drivers = {fl, fr, br, bl};
  ROSDriverAccess ros_drivers(drivers);

  Teleop controller(0, 1.0, 0.01, fl, fr, br, bl);

  while (ros::ok())
  {
    controller.update(left, right);
    ros_drivers.publish();
    loop_rate.sleep();
    ros::spinOnce();
  }
}
