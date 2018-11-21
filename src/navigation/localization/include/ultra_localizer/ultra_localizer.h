#include <localizer/localizer_interface.h>

//This class assumes:
//expected_change is based on the immedietly prior 
// state estimate and is Not the instantaneous derivative.
//measurements are from after the expected_change should have transpired.

//That is, measurement time = last estimate time + time duration of expected_change


class UltraLocalizer : public LocalizerInterface::LocalizerInterface_c
{
public: 
  UltraLocalizer(LocalizerInterface::stateVector gains,
                 LocalizerInterface::stateVector initial_est);

  LocalizerInterface::stateVector getStateVector();
  LocalizerInterface::stateVector getResidual();
  UpdateStatus updateStateVector(double dt); //TODO this needs changed, maybe in interface
  UpdateStatus updateEstimate(LocalizerInterface::stateVector expected_change,
                              LocalizerInterface::stateVector measurements);

//psuedoprotected:
  LocalizerInterface::stateVector diff(LocalizerInterface::stateVector lhs,
                                       LocalizerInterface::stateVector rhs);
  LocalizerInterface::stateVector product(LocalizerInterface::stateVector lhs, 
                                           LocalizerInterface::stateVector rhs);
  LocalizerInterface::stateVector sum(LocalizerInterface::stateVector lhs,
                                      LocalizerInterface::stateVector rhs);

private:
  LocalizerInterface::stateVector estimate;
  LocalizerInterface::stateVector residual;
  LocalizerInterface::stateVector gain;

};