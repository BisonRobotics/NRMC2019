#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dig_control/DigControlAction.h>
#include <dig_control/dig_controller.h>

bool dig_safety, change_dig_state;
actionlib::SimpleActionClient<dig_control::DigControlAction> *client;

void doneCallback(){}
void activeCallback(){}
void feedbackCallback(){}

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  bool rb = joy->buttons[5] == 1; // Dig safety
  bool st = joy->buttons[9] == 1; // Start automatic dig

  dig_safety = rb;
  change_dig_state = st; // Maintain automatic digging until safety is released

  dig_control::DigControlGoal goal;
  if (change_dig_state)
  {
    goal.control_state = (uint8_t)dig_control::ControlState::dig;
    if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      client->sendGoal(goal);
    }
  }
  else
  {
    goal.control_state = (uint8_t)dig_control::ControlState::manual;
    client->sendGoal(goal);
  }

  //ROS_INFO("[teleop] Dv = %i,L = %4.2f, R = %4.2f",
  //         (int)drive_safety, left_wheels, right_wheels);
  //ROS_INFO("[teleop] Dg = %i, C = %4.2f, Bh = %4.1f, Bt = %4.1f, V = %3.1f",
  //         (int)dig_safety, central_duty, backhoe_duty, bucket_duty, vibrator_duty);
}


int main(int argc, char **argv)
{
  dig_safety = false;

  ros::init(argc, argv, "dig_teleop");
  ros::NodeHandle n;
  ros::Subscriber joy_sub = n.subscribe("joy", 2, callback);
  client = new actionlib::SimpleActionClient<dig_control::DigControlAction>("dig_control_server", true);
  ros::Rate rate(50);

  ROS_INFO("Waiting for dig_control_server to start");
  client->waitForServer();
  ROS_INFO("Connected to dig_control_server");
  // Spin until shutdown
  ros::spin();
}
