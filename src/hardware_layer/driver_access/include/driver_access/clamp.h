#ifndef DRIVER_ACCESS_CLAMP_H
#define DRIVER_ACCESS_CLAMP_H

#include <driver_access/limits.h>

namespace driver_access
{
double clamp(double value, double min, double max)
{
  if (max < min)
  {
    throw limits_error("Invalid clamp limits: "
                      + std::to_string(min)
                      + " ≮ "
                      + std::to_string(max));
  }
  if (value > max)
  {
    return max;
  }
  else if (value < min)
  {
    return min;
  }
  else
  {
    return value;
  }
}

double absClamp(double value, double min, double max)
{
  if (value >= 0)
  {
    return clamp(value, min, max);
  }
  else
  {
    return -1.0*clamp(-value, min, max);
  }
}
}

#endif //DRIVER_ACCESS_CLAMP_H
