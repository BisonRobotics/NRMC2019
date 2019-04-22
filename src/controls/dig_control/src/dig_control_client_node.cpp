#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dig_control/dig_control_client.h>

using namespace dig_control;

bool dig_safety, change_dig_state, change_dump_state;
DigControlClient *client;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  bool rb = joy->buttons[5] == 1; // Dig safety
  bool bt = joy->buttons[8] == 1; // Start automatic dump
  bool st = joy->buttons[9] == 1; // Start automatic dig

  dig_safety = rb;
  change_dig_state = st; // Maintain automatic digging until safety is released
  change_dump_state = bt;

  if (dig_safety)
  {
    if (change_dig_state)
    {
      ROS_INFO("[callback] %s", to_string(client->getControlState()).c_str());
      switch (client->getControlState())
      {
        case ControlState::dig:
        {
          client->setControlState(ControlState::finish_dig);
          break;
        }
        default:
        {
          client->setControlState(ControlState::dig);
          break;
        }
      }
    }
    else if (change_dump_state)
    {
      ROS_INFO("[callback] %s", to_string(client->getControlState()).c_str());
      client->setControlState(ControlState::dump);
    }
  }
  else
  {
    if (client->getControlState() != ControlState::manual)
    {
      client->setControlState(ControlState::manual);
    }
  }

  //ROS_INFO("[teleop] Dv = %i,L = %4.2f, R = %4.2f",
  //         (int)drive_safety, left_wheels, right_wheels);
  //ROS_INFO("[teleop] Dg = %i, C = %4.2f, Bh = %4.1f, Bt = %4.1f, V = %3.1f",
  //         (int)dig_safety, central_duty, backhoe_duty, bucket_duty, vibrator_duty);
}


int main(int argc, char **argv)
{
  dig_safety = false;

  ros::init(argc, argv, "dig_control_client");
  ros::NodeHandle nh;
  ros::Subscriber joy_sub = nh.subscribe("joy", 2, callback);
  client = new DigControlClient;
  ros::spin();
}
