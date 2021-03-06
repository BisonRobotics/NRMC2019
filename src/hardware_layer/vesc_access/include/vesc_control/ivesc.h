#ifndef __VESC_INTERFACE_H_
#define __VESC_INTERFACE_H_
class iVesc
{
public:
  //    virtual ~iVesc();
  //    virtual iVesc();
  virtual void setRpm(float rpm) = 0;
  virtual void setCurrent(float current) = 0;
  virtual float getCurrent(void) = 0;
  virtual int getRpm(void) = 0;
  virtual int getADC(void) = 0;
  virtual bool getForLimit(void) = 0;
  virtual bool getRevLimit(void) = 0;
  virtual void setDuty(float duty) = 0;
  virtual void setCustom(float setpoint) = 0;
  virtual void setCustom(float setpoint, uint index) = 0;
  virtual float getInCurrent() = 0;
  virtual int getTachometer() = 0;
  virtual float getVin() = 0;
  virtual bool encoderIndexFound() = 0;
  virtual bool isAlive() = 0;
};

class VescException : public std::runtime_error
{
public:
  explicit VescException(const char* msg) : std::runtime_error(msg)
  {
  }
  explicit VescException(std::string msg) : std::runtime_error(msg){

  }
};
#endif
