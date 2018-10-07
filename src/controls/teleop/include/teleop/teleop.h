#include <driver_access/driver_access.h>

namespace teleop
{
class Teleop
{
  public:

    Teleop(double min, double max, double deadzone,
        driver_access::DriverAccess *fl, driver_access::DriverAccess *fr,
        driver_access::DriverAccess *br, driver_access::DriverAccess *bl);

    void update(double left, double right);
    void stopMotors();
    void updateLimits(double min, double max, double deadzone);

    double scale(double value);
    static double clamp(double value);

  private:
    double min, max, deadzone;
    driver_access::DriverAccess *fl, *fr, *br, *bl;

};
}
