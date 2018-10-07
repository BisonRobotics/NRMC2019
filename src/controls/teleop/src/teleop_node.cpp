#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <teleop/teleop.h>
#include <vrep_driver_access/vrep_driver_access.h>

using namespace teleop;
using driver_access::Limits;
using driver_access::Mode;
using driver_access::DriverAccess;
using driver_access::DriverAccessPtr;
using driver_access::VREPDriverAccess;

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
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, callback);
  ros::Rate loop_rate(10);

  bool sim;
  nh.param("sim", sim, true);

  Limits limits(0, 0, 0, 1, 0, 1);
  DriverAccessPtr fl, fr, br, bl;
  if (sim)
  {
    fl.reset(new VREPDriverAccess(limits, 0, Mode::effort));
    fr.reset(new VREPDriverAccess(limits, 1, Mode::effort));
    br.reset(new VREPDriverAccess(limits, 2, Mode::effort));
    bl.reset(new VREPDriverAccess(limits, 3, Mode::effort));
  }
  else
  {
    // VESC implementation goes here
  }

  Teleop controller(0, 0.2, 0.01, fl, fr, br, bl);

  while (ros::ok())
  {
    controller.update(left, right);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
