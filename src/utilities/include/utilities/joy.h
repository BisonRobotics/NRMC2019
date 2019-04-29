#ifndef COMPETITION_JOY_H
#define COMPETITION_JOY_H

#include <sensor_msgs/Joy.h>

namespace utilities
{
  class Joy : public sensor_msgs::Joy
  {
  public:
    Joy(); // Implicitly calls default constructor of parent class
    Joy(const sensor_msgs::JoyConstPtr &joy);

    enum Button // Twelve actual buttons + 4 axis buttons
    {
      // Physical mappings
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
      PD = 15,  // Pad down press
      // Manual state mappings
      MANUAL_SAFETY = LB,
      BUCKET_DOWN = X,
      BUCKET_UP = Y,
      LINEAR_IN = A,
      LINEAR_OUT = B,
      VIBRATOR_ON = RT,
      VIBRATOR_OFF = LT,
      CENTRAL_DRIVE_UP = PU,
      CENTRAL_DRIVE_DOWN = PD,
      // Autonomous state mappings
      AUTONOMY_SAFETY = RB,
      DIG = Y,
      DUMP = X,
      START_PATH = B,
      CLEAR_WAYPOINTS = A,
      FOLLOW_ROBOT = PL,
      UNFOLLOW_ROBOT = PR,
      FORWARD = PU,
      REVERSE = PD,
      START_COMPETITION = ST,
      STOP_COMPETITION = BK
    };

    enum Axis // Six axis
    {
      // Physical mappings
      LY = 0, // Left stick y-axis
      LX = 1, // Left stick x-axis
      RY = 2, // Right stick y-axis
      RX = 3, // Right stick x-axis
      PY = 4, // Pad x-axis
      PX = 5,  // Pad y-axis
      // Manual state mappings
      TELEOP_LEFT = LX,
      TELEOP_RIGHT = RX
      // Autonomy state mappings
    };

    bool get(Button button);
    double get(Axis axis);
  };
}

#endif //COMPETITION_CONTROLLER_JOY_H
