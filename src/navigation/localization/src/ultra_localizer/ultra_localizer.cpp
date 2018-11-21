#include <ultra_localizer/ultra_localizer.h>

UltraLocalizer::UltraLocalizer(LocalizerInterface::stateVector gains,
               LocalizerInterface::stateVector initial_est)
:gain(gains), estimate(initial_est)
{
}




LocalizerInterface::stateVector UltraLocalizer::getStateVector()
{
  return estimate;
}


LocalizerInterface::stateVector UltraLocalizer::getResidual()
{
  return residual;
}

UltraLocalizer::UpdateStatus  UltraLocalizer::updateStateVector(double dt) 
{
  /*not implemented*/
  /*
  int * vargin;
  int * vargin_2;
  vargin_2 = * vargin;
  memcpy(vargin_2, vargin, vargin);
  */
  return UltraLocalizer::UpdateStatus::UPDATE_FAILED_SENSOR_ERROR;
}

UltraLocalizer::UpdateStatus  UltraLocalizer::updateEstimate(LocalizerInterface::stateVector expected_change,
                                             LocalizerInterface::stateVector measurements) 
{
  residual = diff(measurements, sum(estimate, expected_change));
  estimate = sum(estimate, sum(expected_change, product(gain, residual)));
  return UltraLocalizer::UpdateStatus::UPDATE_SUCCESS;
}

LocalizerInterface::stateVector UltraLocalizer::diff(LocalizerInterface::stateVector lhs,
                                                     LocalizerInterface::stateVector rhs)
{
  LocalizerInterface::stateVector ret;
  ret.alpha = lhs.alpha - rhs.alpha;
  ret.omega = lhs.omega - rhs.omega;
  ret.theta = lhs.theta - rhs.theta;

  ret.x_accel = lhs.x_accel - rhs.x_accel;
  ret.y_accel = lhs.y_accel - rhs.y_accel;
  ret.x_vel = lhs.x_vel - rhs.x_vel;
  ret.y_vel = lhs.y_vel - rhs.y_vel;
  ret.x_pos = lhs.x_pos - rhs.x_pos;
  ret.y_pos = lhs.y_pos - rhs.y_pos;

  return ret;
}

LocalizerInterface::stateVector UltraLocalizer::product(LocalizerInterface::stateVector lhs,
                                                        LocalizerInterface::stateVector rhs)
{
  LocalizerInterface::stateVector ret;
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

LocalizerInterface::stateVector UltraLocalizer::sum(LocalizerInterface::stateVector lhs,
                                                    LocalizerInterface::stateVector rhs)
{
  LocalizerInterface::stateVector ret;
  ret.alpha = lhs.alpha + rhs.alpha;
  ret.omega = lhs.omega + rhs.omega;
  ret.theta = lhs.theta + rhs.theta;

  ret.x_accel = lhs.x_accel + rhs.x_accel;
  ret.y_accel = lhs.y_accel + rhs.y_accel;
  ret.x_vel = lhs.x_vel + rhs.x_vel;
  ret.y_vel = lhs.y_vel + rhs.y_vel;
  ret.x_pos = lhs.x_pos + rhs.x_pos;
  ret.y_pos = lhs.y_pos + rhs.y_pos;

  return ret;
}
