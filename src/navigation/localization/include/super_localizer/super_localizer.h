#include <localizer/localizer.h>
#include <sensor_access/imu_sensor_interface.h>
#include <sensor_access/pos_sensor_interface.h>
#include <readable_sensors/readable_sensors.h>
#include <super_localizer/super_localizer_helper.h>

#define XRESGAIN .02f  // trust sensors -> higher gain
#define YRESGAIN .003f
#define THETARESGAIN .02f
#define DXRESGAIN .0075f
#define DYRESGAIN .0100f
#define OMEGARESGAIN .005f
#define DX2RESGAIN .2f
#define DY2RESGAIN .2f
#define ALPHARESGAIN .005f

class SuperLocalizer : public LocalizerInterface::LocalizerInterface_c
{
public:
  SuperLocalizer(double axleLen, double xi, double yi, double thi, iVescAccess *frontLeftVesc,
                 iVescAccess *frontRightVesc, iVescAccess *backRightVesc, iVescAccess *backLeftVesc,
                 ImuSensorInterface *centerIMU, PosSensorInterface *posSensor, LocalizerInterface::StateVector gains);
  SuperLocalizer(double axleLen, double xi, double yi, double thi, iVescAccess *frontLeftVesc,
                 iVescAccess *frontRightVesc, iVescAccess *backRightVesc, iVescAccess *backLeftVesc,
                 PosSensorInterface *posSensor, LocalizerInterface::StateVector gains);
  UpdateStatus updateStateVector(double dt);
  ~SuperLocalizer();

  // static constexpr Localizer::stateVector_s default_gains;
  LocalizerInterface::StateVector getStateVector();
  uint8_t getNumSensors();
  bool getHaveImu();
  bool getHavePosition();
  LocalizerInterface::StateVector getResidual();
  LocalizerInterface::StateVector getMeasured();
  LocalizerInterface::StateVector getGainVector();
  bool getIsDataGood(void);

private:
  bool data_is_good;
  bool imu_is_good;
  bool pos_is_good;
  void setDataIsGood (void);
  Localizer *deadReck;
  ImuSensorInterface *cIMU;
  PosSensorInterface *pSensor;
  ReadableSensors *sensors[2];
  uint8_t num_sensors;
  bool have_imu;
  bool have_pos;
  // LocalizerInterface::stateVector initState ();
  LocalizerInterface::StateVector residual;
  LocalizerInterface::StateVector measured;
  LocalizerInterface::StateVector gainVector;
  static constexpr unsigned int position_sensor_index = 0;
  static constexpr unsigned int imu_sensor_index =1;
  LocalizerInterface::StateVector state_vector;

  LocalizerInterface::StateVector initState(double xi, double yi, double theta);
};

// I couldn't figure out how to make this a static class member. I tried quite a few things...
// apparently, the declaration is supposed to go in this header and an actual definition in the
//.cpp file, but I couldn't find the right way to do that... so here it is instead.
static constexpr LocalizerInterface::StateVector SuperLocalizer_default_gains = {.x_pos = XRESGAIN,
                                                                                 .y_pos = YRESGAIN,
                                                                                 .theta = THETARESGAIN,
                                                                                 .x_vel = DXRESGAIN,
                                                                                 .y_vel = DYRESGAIN,
                                                                                 .omega = OMEGARESGAIN,
                                                                                 .x_accel = DX2RESGAIN,
                                                                                 .y_accel = DY2RESGAIN,
                                                                                 .alpha = ALPHARESGAIN };
