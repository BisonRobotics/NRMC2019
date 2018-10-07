#include <driver_access/driver_access.h>

namespace teleop
{
class Teleop
{
  public:

    Teleop(double min, double max, double deadzone,
        driver_access::DriverAccessPtr fl, driver_access::DriverAccessPtr fr,
        driver_access::DriverAccessPtr br, driver_access::DriverAccessPtr bl);

    void update(double left, double right);
    void updateLimits(double min, double max, double deadzone);
    void stopMotors();

    double scale(double value);
    static double clamp(double value);

  private:
    double min, max, deadzone;
    driver_access::DriverAccessPtr fl, fr, br, bl;

};
}
