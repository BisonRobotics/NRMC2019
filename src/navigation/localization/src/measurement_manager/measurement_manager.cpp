#include <measurement_manager/measurement_manager.h>

MeasurementManager::MeasurementManager()
{
  x_vel = 0; //these get reset if IMU is below dead_zone
  y_vel = 0;
  dz = 0;
}

bool MeasurementManager::giveImu(ImuSensorInterface *IMU_in, double x_offset,
                                         double y_offset, double dead_zone)
{
  IMU = IMU_in;
  x_off = x_offset;
  y_off = y_offset;
  dz = dead_zone;
  x_vel = 0;
  y_vel = 0;
}

bool MeasurementManager::givePos(PosSensorInterface *posSensor_in)
{
  pSensor = posSensor_in;
}

LocalizerInterface::StateVector MeasurementManager::getMeasured(double dt)
{
  // read and integrate IMU
  //TODO transform using offset
  //TODO estimate vel from POS and if <dz, zero out integrators
  x_vel += IMU->getX() * dt;
  y_vel += IMU->getY() * dt;
  //read POS

  LocalizerInterface::StateVector ret;
  ret.alpha = 0;
  ret.x_accel = 0;
  ret.y_accel = 0;
  ret.omega = IMU->getOmega();
  ret.x_vel = x_vel;
  ret.y_vel = y_vel;
  ret.theta = pSensor->getTheta();
  ret.x_pos = pSensor->getX();
  ret.y_pos = pSensor->getY();

  return ret;
}