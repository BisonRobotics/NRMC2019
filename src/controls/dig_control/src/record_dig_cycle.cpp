#include <ros/ros.h>
#include <dig_control/Debug.h>
#include <dig_control/dig_control_client.h>

#include <iostream>
#include <fstream>
#include <string>

using namespace dig_control;

std::ofstream *file;

void callback(const dig_control::Debug::ConstPtr &message)
{
  *file << std::to_string(message->duty.bucket)           << "," << std::to_string(message->duty.backhoe)     << ","
        << std::to_string(message->duty.central)          << "," << std::to_string(message->duty.vibrator)    << ","
        << std::to_string(message->state_i.bucket)        << "," << std::to_string(message->state_i.backhoe)  << ","
        << std::to_string(message->state_i.central_drive) << "," << std::to_string(message->state_i.dig)      << ","
        << std::to_string(message->state_i.control)       << "," << std::to_string(message->central_position) << "\n";
}


int main(int argc, char **argv)
{
  file = new std::ofstream;
  file->open("dig_cycle.csv");

  ros::init(argc, argv, "record_dig_cycle");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("dig_control_server/debug", 10, callback);
  ros::spin();

  file->close();
}


