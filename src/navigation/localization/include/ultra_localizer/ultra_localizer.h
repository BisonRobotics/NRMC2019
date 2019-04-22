#include <localizer/localizer_interface.h>

//This class assumes:
//expected_change is based on the immedietly prior 
// state estimate and is Not the instantaneous derivative.
//measurements are from after the expected_change should have transpired.

//That is, measurement time = last estimate time + time duration of expected_change

#define XRESGAIN .05f  // trust sensors -> higher gain
#define YRESGAIN .05f
#define THETARESGAIN .05f
#define DXRESGAIN .0075f
#define DYRESGAIN .0100f
#define OMEGARESGAIN .005f
#define DX2RESGAIN .2f
#define DY2RESGAIN .2f
#define ALPHARESGAIN .005f

class UltraLocalizer : public LocalizerInterface::LocalizerInterface_c
{
public: 
  UltraLocalizer(LocalizerInterface::StateVector gains,
                 LocalizerInterface::StateVector initial_est);

  LocalizerInterface::StateVector getStateVector();
  LocalizerInterface::StateVector getResidual();
  UpdateStatus updateStateVector(double dt); //TODO this needs changed, maybe in interface
  UpdateStatus updateEstimate(LocalizerInterface::StateVector expected_change,
                              LocalizerInterface::StateVector measurements);

//psuedoprotected:
  LocalizerInterface::StateVector diff(LocalizerInterface::StateVector lhs,
                                       LocalizerInterface::StateVector rhs);
  LocalizerInterface::StateVector product(LocalizerInterface::StateVector lhs,
                                           LocalizerInterface::StateVector rhs);
  LocalizerInterface::StateVector sum(LocalizerInterface::StateVector lhs,
                                      LocalizerInterface::StateVector rhs);

private:
  LocalizerInterface::StateVector estimate;
  LocalizerInterface::StateVector residual;
  LocalizerInterface::StateVector gain;

};

//Theres a better way. maybe these need a namespace?
static constexpr LocalizerInterface::StateVector UltraLocalizer_default_gains = {.x_pos = XRESGAIN,
                                                                                 .y_pos = YRESGAIN,
                                                                                 .theta = THETARESGAIN,
                                                                                 .x_vel = DXRESGAIN,
                                                                                 .y_vel = DYRESGAIN,
                                                                                 .omega = OMEGARESGAIN,
                                                                                 .x_accel = DX2RESGAIN,
                                                                                 .y_accel = DY2RESGAIN,
                                                                                 .alpha = ALPHARESGAIN };

static constexpr LocalizerInterface::StateVector UltraLocalizer_initial_estimate = {.x_pos = 1.2,
                                                                                 .y_pos = 0,
                                                                                 .theta = 0,
                                                                                 .x_vel = 0,
                                                                                 .y_vel = 0,
                                                                                 .omega = 0,
                                                                                 .x_accel = 0,
                                                                                 .y_accel = 0,
                                                                                 .alpha = 0 };

static constexpr LocalizerInterface::StateVector UltraLocalizer_zero_vector = {.x_pos = 0,
                                                                                 .y_pos = 0,
                                                                                 .theta = 0,
                                                                                 .x_vel = 0,
                                                                                 .y_vel = 0,
                                                                                 .omega = 0,
                                                                                 .x_accel = 0,
                                                                                 .y_accel = 0,
                                                                                 .alpha = 0 };
