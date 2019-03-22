#include <ros/ros.h>
#include <stepper/stepper.h>

using namespace stepper;

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "stepper_node");
  ros::NodeHandle nh;

  Stepper stepper("can0", 1, 11);

  while(ros::ok())
  {
    if (stepper.messagesAvailable())
    {
      can_frame_t rx_frame = stepper.receiveFrame();
      int32_t value;
      memcpy(&value, &rx_frame.data, 2);
      ROS_INFO("[%i] %i", rx_frame.can_id, value);
      can_frame_t tx_frame;
      tx_frame.can_id = 1;
      tx_frame.can_dlc = 2;
      tx_frame.data[0] = 10;
      tx_frame.data[1] = 11;
      stepper.sendFrame(tx_frame);
    }
    else
    {
      //ROS_INFO("No messages");
      usleep(1000);
    }
  }

  return 0;
}