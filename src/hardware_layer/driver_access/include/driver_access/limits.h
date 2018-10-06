#ifndef DRIVER_ACCESS_LIMITS_H
#define DRIVER_ACCESS_LIMITS_H

#include <stdexcept>

namespace driver_access
{

struct limits_error : public std::logic_error
{
  limits_error(std::string const &message) : std::logic_error(message) {};
};

class Limits
{
  public:
    const double min_velocity;
    const double max_velocity;
    const double min_torque;
    const double max_torque;
    const double min_position;
    const double max_position;

    Limits(double min_velocity, double max_velocity,
           double min_torque, double max_torque,
           double min_position, double max_position) :
        min_velocity(min_velocity), max_velocity(max_velocity),
        min_torque(min_torque), max_torque(max_torque),
        min_position(min_position), max_position(max_position)
    {
      check();
    }

    Limits(const Limits &limits) :
        min_velocity(limits.min_velocity), max_velocity(limits.max_velocity),
        min_torque(limits.min_torque), max_torque(limits.max_torque),
        min_position(limits.min_position), max_position(limits.max_position)
    {};


  private:
    void check()
    {
      if (max_velocity < min_velocity)
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
      else if (max_torque < min_torque)
      {
        throw limits_error("Invalid torque limits: "
                          + std::to_string(min_velocity)
                          + " ≮ "
                          + std::to_string(max_velocity));
      }
      else if (max_torque < 0 || min_torque < 0)
      {
        throw limits_error("Torque limits must be positive: max = "
                          + std::to_string(max_torque)
                          + ", min = "
                          + std::to_string(min_torque));
      }
      else if (max_position < min_position)
      {
        throw limits_error("Invalid position limits: "
                          + std::to_string(min_velocity)
                          + " ≮ "
                          + std::to_string(max_velocity));
      }
    }
};
}

#endif //DRIVER_ACCESS_LIMITS_H
