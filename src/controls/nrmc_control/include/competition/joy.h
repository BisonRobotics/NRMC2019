#ifndef COMPETITION_JOY_H
#define COMPETITION_JOY_H

#include <sensor_msgs/Joy.h>

namespace competition
{
  class Joy
  {
  public:
    enum class Button // Twelve actual buttons + 4 axis buttons
    {
      x =   0, // x button
      a =   1, // a button
      b =   2, // b button
      y =   3, // y button
      lb =  4  // Left bumper
      rb =  5, // Right bumper
      lt =  6, // Left trigger
      rt =  7, // Right trigger
      bk =  8, // Back button
      st =  9, // Start button
      ls = 10, // Left stick press
      lr = 11, // Right stick press
      pl = 12, // Pad left press
      pr = 13, // Pad right press
      pu = 14, // Pad up press
      pd = 15  // Pad down press
    };

    enum class Axis // Six axis
    {
      ly = 0, // Left stick y-axis
      lx = 1, // Left stick x-axis
      ry = 2, // Right stick y-axis
      rx = 3, // Right stick x-axis
      px = 4, // Pad x-axis
      py = 5  // Pad y-axis
    };

    Joy(const sensor_msgs::JoyConstPtr &msg);

    bool get(Button button);
    double get(Axis axis);

  private:
    sensor_msgs::Joy msg;
  };
}

#endif //COMPETITION_CONTROLLER_JOY_H
