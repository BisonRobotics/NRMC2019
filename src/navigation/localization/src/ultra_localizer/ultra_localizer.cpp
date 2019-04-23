#include <ultra_localizer/ultra_localizer.h>
#include <cmath>

UltraLocalizer::UltraLocalizer(LocalizerInterface::StateVector gains,
               LocalizerInterface::StateVector initial_est)
:gain(gains), estimate(initial_est)
{
}

double angleDiff(double angle1, double angle2)
{
  double diff = std::atan2(std::sin(angle1 - angle2), std::cos(angle1 - angle2));
  return diff;
}


LocalizerInterface::StateVector UltraLocalizer::getStateVector()
{
  return estimate;
}


LocalizerInterface::StateVector UltraLocalizer::getResidual()
{
  return residual;
}

UltraLocalizer::UpdateStatus  UltraLocalizer::updateStateVector(double dt) 
{
  /*not implemented*/ //This method is here temproarily while interfaces
  /*                  // are transitioned.
  int * vargin; 
  int * vargin_2;
  vargin_2 = * vargin;
  memcpy(vargin_2, vargin, vargin);
  */
  return UltraLocalizer::UpdateStatus::UPDATE_FAILED_SENSOR_ERROR;
}

UltraLocalizer::UpdateStatus  UltraLocalizer::updateEstimate(LocalizerInterface::StateVector expected_change,
                                             LocalizerInterface::StateVector measurements)
{
  residual = diff(measurements, sum(estimate, expected_change));
  estimate = sum(estimate, sum(expected_change, product(gain, residual)));
  return UltraLocalizer::UpdateStatus::UPDATE_SUCCESS;
}

LocalizerInterface::StateVector UltraLocalizer::diff(LocalizerInterface::StateVector lhs,
                                                     LocalizerInterface::StateVector rhs)
{
  LocalizerInterface::StateVector ret;
  ret.alpha = lhs.alpha - rhs.alpha;
  ret.omega = lhs.omega - rhs.omega;
  ret.theta = angleDiff(lhs.theta, rhs.theta);

  ret.x_accel = lhs.x_accel - rhs.x_accel;
  ret.y_accel = lhs.y_accel - rhs.y_accel;
  ret.x_vel = lhs.x_vel - rhs.x_vel;
  ret.y_vel = lhs.y_vel - rhs.y_vel;
  ret.x_pos = lhs.x_pos - rhs.x_pos;
  ret.y_pos = lhs.y_pos - rhs.y_pos;

  return ret;
}

LocalizerInterface::StateVector UltraLocalizer::product(LocalizerInterface::StateVector lhs,
                                                        LocalizerInterface::StateVector rhs)
{
  LocalizerInterface::StateVector ret;
  ret.alpha = lhs.alpha * rhs.alpha;
  ret.omega = lhs.omega * rhs.omega;
  ret.theta = lhs.theta * rhs.theta;
  ret.x_accel = lhs.x_accel * rhs.x_accel;
  ret.y_accel = lhs.y_accel * rhs.y_accel;
  ret.x_vel = lhs.x_vel * rhs.x_vel;
  ret.y_vel = lhs.y_vel * rhs.y_vel;
  ret.x_pos = lhs.x_pos * rhs.x_pos;
  ret.y_pos = lhs.y_pos * rhs.y_pos;

  return ret;
}

LocalizerInterface::StateVector UltraLocalizer::sum(LocalizerInterface::StateVector lhs,
                                                    LocalizerInterface::StateVector rhs)
{
  LocalizerInterface::StateVector ret;
  ret.alpha = lhs.alpha + rhs.alpha;
  ret.omega = lhs.omega + rhs.omega;
  ret.theta = lhs.theta + rhs.theta;
  ret.theta = angleDiff(ret.theta, 0);

  ret.x_accel = lhs.x_accel + rhs.x_accel;
  ret.y_accel = lhs.y_accel + rhs.y_accel;
  ret.x_vel = lhs.x_vel + rhs.x_vel;
  ret.y_vel = lhs.y_vel + rhs.y_vel;
  ret.x_pos = lhs.x_pos + rhs.x_pos;
  ret.y_pos = lhs.y_pos + rhs.y_pos;

  return ret;
}
