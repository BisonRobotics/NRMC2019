#ifndef WAYPOINT_CONTROL_WAYPOINT_CONFIG_H
#define WAYPOINT_CONTROL_WAYPOINT_CONFIG_H

#include <boost/math/constants/constants.hpp>

namespace waypoint_control
{
  using boost::math::double_constants::pi;

  class Config
  {
  public:
    Config() :
      initial_angular_variation(10.0*pi/180.0),
      max_angular_velocity(0.05)
    {}

    double initial_angular_variation;
    double max_angular_velocity;
  private:
  };
}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONFIG_H
