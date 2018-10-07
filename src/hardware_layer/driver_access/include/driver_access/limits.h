#ifndef DRIVER_ACCESS_LIMITS_H
#define DRIVER_ACCESS_LIMITS_H

#include <driver_access/driver_access_error.h>

namespace driver_access
{

struct limits_error : public driver_access_error
{
  limits_error(std::string const &message) : driver_access_error(message) {};
};

class Limits
{
  public:
    const double min_position;
    const double max_position;
    const double min_velocity;
    const double max_velocity;
    const double min_effort;
    const double max_effort;

    Limits(double min_position, double max_position,
           double min_velocity, double max_velocity,
           double min_effort, double max_effort) :
        min_position(min_position), max_position(max_position),
        min_velocity(min_velocity), max_velocity(max_velocity),
        min_effort(min_effort), max_effort(max_effort)
    {
      check();
    }

    Limits(const Limits &limits) :
        min_position(limits.min_position), max_position(limits.max_position),
        min_velocity(limits.min_velocity), max_velocity(limits.max_velocity),
        min_effort(limits.min_effort), max_effort(limits.max_effort)
    {};


  private:
    void check()
    {
      if (max_position < min_position)
      {
        throw limits_error("Invalid position limits: "
                           + std::to_string(min_velocity)
                           + " ≮ "
                           + std::to_string(max_velocity));
      }
      else if (max_velocity < min_velocity)
      {
        throw limits_error("Invalid velocity limits: "
                          + std::to_string(min_velocity)
                          + " ≮ "
                          + std::to_string(max_velocity));
      }
      else if (max_velocity < 0 || min_velocity < 0)
      {
        throw limits_error("Velocity limits must be positive: max = "
                          + std::to_string(max_velocity)
                          + ", min = "
                          + std::to_string(min_velocity));
      }
      else if (max_effort < min_effort)
      {
        throw limits_error("Invalid effort limits: "
                          + std::to_string(min_effort)
                          + " ≮ "
                          + std::to_string(max_effort));
      }
      else if (max_effort < 0 || min_effort < 0)
      {
        throw limits_error("Effort limits must be positive: max = "
                          + std::to_string(max_effort)
                          + ", min = "
                          + std::to_string(min_effort));
      }
    }
};
}

#endif //DRIVER_ACCESS_LIMITS_H
