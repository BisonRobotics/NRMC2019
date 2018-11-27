#include <sensor_access/imu_sensor_interface.h>
#include <sensor_access/pos_sensor_interface.h>
#include <readable_sensors/readable_sensors.h>
#include <localizer/localizer_interface.h>

//Given an IMU, populates x,y velocity and omega
//Given a  POS, populates x,y and theta

class MeasurementManager
{
public:
  MeasurementManager();
  bool giveImu(ImuSensorInterface *IMU_in, double x_offset, double y_offset, double dead_zone);
  bool givePos(PosSensorInterface *posSensor_in);
  LocalizerInterface::stateVector getMeasured(double dt);
private:
  ImuSensorInterface *IMU;
  double x_off;
  double y_off;
  double dz;

  double x_vel; //these get reset if IMU is below dead_zone
  double y_vel;

  PosSensorInterface *pSensor;
};