#ifndef COMPETITION_JOY_H
#define COMPETITION_JOY_H

#include <sensor_msgs/Joy.h>

namespace competition
{
  class Joy : public sensor_msgs::Joy
  {
  public:
    Joy(); // Implicitly calls default constructor of parent class
    Joy(const sensor_msgs::JoyConstPtr &joy);

    enum Button // Twelve actual buttons + 4 axis buttons
    {
      X =   0, // x button
      A =   1, // a button
      B =   2, // b button
      Y =   3, // y button
      LB =  4, // Left bumper
      RB =  5, // Right bumper
      LT =  6, // Left trigger
      RT =  7, // Right trigger
      BK =  8, // Back button
      ST =  9, // Start button
      LS = 10, // Left stick press
      LR = 11, // Right stick press
      PL = 12, // Pad left press
      PR = 13, // Pad right press
      PU = 14, // Pad up press
      PD = 15  // Pad down press
    };

    enum Axis // Six axis
    {
      LY = 0, // Left stick y-axis
      LX = 1, // Left stick x-axis
      RY = 2, // Right stick y-axis
      RX = 3, // Right stick x-axis
      PX = 4, // Pad x-axis
      PY = 5  // Pad y-axis
    };

    enum CompetitionMappings
    {
      MANUAL = LB,
      AUTONOMY = RB
    };

    bool get(Button button);
    double get(Axis axis);
  };
}

#endif //COMPETITION_CONTROLLER_JOY_H
