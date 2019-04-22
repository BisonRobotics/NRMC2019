#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <drive_controller/drive_control_client.h>

using namespace drive_controller;

bool safety;
DriveControlClient *client;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  bool lb = joy->buttons[4] == 1; // Safety

  safety = lb;

  if (!safety)
  {
    if (client->getControlState() != ControlState::manual)
    {
      client->setControlState(ControlState::manual);
    }
  }
}


int main(int argc, char **argv)
{
  safety = false;

  ros::init(argc, argv, "drive_control_client");
  ros::NodeHandle nh;
  ros::Subscriber joy_sub = nh.subscribe("joy", 2, callback);
  client = new DriveControlClient;
  ros::spin();
}
