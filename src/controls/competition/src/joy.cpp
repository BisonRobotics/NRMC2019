#include <competition/joy.h>

using namespace competition;

bool Joy::get(Button button)
{
  switch(button)
  {
    case pl:
      return get(py) < 0.0;
    case pr:
      return get(py) > 0.0;
    case pu:
      return get(px) > 0.0;
    case pd:
      return get(px) < 0.0;
    default:
      return this->buttons[button] == 1;
  }
}

double Joy::get(Axis axis)
{
  return this->axes[axis];
}