#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>



class TeleopInterface
{
public:
  enum Mode
  {
    velocity,
    duty
  };

  TeleopInterface(Mode mode, float max_value, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl);
  explicit TeleopInterface(Mode mode, float max_value);

  void setMax(float value);
  float getMax();
  void setMode(Mode mode);
  Mode getMode();
  void stopMotors();
  void update(float left, float right);
  ~TeleopInterface();
  iVescAccess *fl, *fr, *br, *bl;

private:
  Mode mode;
  float max_value;
  bool internally_alloc;
};
