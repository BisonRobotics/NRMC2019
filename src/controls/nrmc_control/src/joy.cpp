#include <competition/joy.h>

using namespace competition;

Joy::Joy(const sensor_msgs::JoyConstPtr &msg) : msg(*msg){}

bool Joy::get(competition_controller::Joy::Button button)
{
  return false;
}

double Joy::get(competition_controller::Joy::Axis axis)
{
  return 0;
}