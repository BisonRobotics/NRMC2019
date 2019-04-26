#include <utilities/joy.h>

using namespace utilities;

Joy::Joy()
{}

Joy::Joy(const sensor_msgs::JoyConstPtr &joy)
{
  this->header = joy->header;
  this->buttons = joy->buttons;
  this->axes = joy->axes;
}

bool Joy::get(Button button)
{
  switch(button)
  {
    case PL:
      return get(PY) > 0.0;
    case PR:
      return get(PY) < 0.0;
    case PU:
      return get(PX) > 0.0;
    case PD:
      return get(PX) < 0.0;
    default:
      return this->buttons[button] == 1;
  }
}

double Joy::get(Axis axis)
{
  return this->axes[axis];
}
