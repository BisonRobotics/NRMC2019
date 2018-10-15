#include <teleop/teleop.h>

using namespace teleop;

using driver_access::DriverAccess;
using std::abs;

Teleop::Teleop(double min, double max, double deadzone,
    DriverAccess *fl, DriverAccess *fr, DriverAccess *br, DriverAccess *bl)
{
  this->fl = fl;
  this->fr = fr;
  this->br = br;
  this->bl = bl;

  updateLimits(min, max, deadzone);
  stopMotors();
}

void Teleop::update(double left, double right)
{
  left = clamp(left);
  right = clamp(right);

  if (abs(left) > deadzone)
  {
    left = scale(left);
    fl->setPoint(left);
    bl->setPoint(left);
  }
  else
  {
    // TODO this should release the motors
    fl->setPoint(0);
    bl->setPoint(0);
  }

  if (abs(right) > deadzone)
  {
    right = scale(right);
    fr->setPoint(right);
    br->setPoint(right);
  }
  else
  {
    fr->setPoint(0);
    br->setPoint(0);
  }
}


void Teleop::stopMotors()
{
  fl->setPoint(0);
  fr->setPoint(0);
  br->setPoint(0);
  bl->setPoint(0);
}

void Teleop::updateLimits(double min, double max, double deadzone)
{
  if (max < min)
  {
    throw std::logic_error("Max value must be greater than min value");
  }
  else if (max < 0.0 || min < 0.0 || deadzone < 0.0)
  {
    throw std::logic_error("Limit values must be positive");
  }
  else if (deadzone > 1.0)
  {
    throw std::logic_error("Deadzone must be less than 1");
  }
  else
  {
    this->min = min;
    this->max = max;
    this->deadzone = deadzone;
  }
}

double Teleop::scale(double value)
{
  return ((value - deadzone) / (1.0 - deadzone)) * (max - min) + min;
}

double Teleop::clamp(double value)
{
  if (value > 1.0)
  {
    return 1.0;
  }
  else if (value < -1.0)
  {
    return -1.0;
  }
  else
  {
    return value;
  }
}