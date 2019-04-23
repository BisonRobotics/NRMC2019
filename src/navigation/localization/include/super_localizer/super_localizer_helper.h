#include <localizer/localizer.h>
namespace LocalizerInterface
{
LocalizerInterface::StateVector diff(LocalizerInterface::StateVector lhs, LocalizerInterface::StateVector rhs);
LocalizerInterface::StateVector multiply(LocalizerInterface::StateVector lhs, LocalizerInterface::StateVector rhs);
LocalizerInterface::StateVector addFromModel(LocalizerInterface::StateVector lhs, LocalizerInterface::StateVector rhs,
                                             double dt, bool imu);
LocalizerInterface::StateVector initState(double xi, double yi, double theta);
}
