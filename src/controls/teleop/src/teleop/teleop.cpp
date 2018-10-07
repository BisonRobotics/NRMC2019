#include <teleop/teleop.h>

using namespace teleop;

using driver_access::DriverAccessPtr;
using std::abs;

Teleop::Teleop(double min, double max, double deadzone,
    DriverAccessPtr fl, DriverAccessPtr fr, DriverAccessPtr br, DriverAccessPtr bl)
{
  this->fl = fl;
  this->fr = fr;
  this->br = br;
  this->bl = bl;

  updateLimits(min, max, deadzone);
  stopMotors();
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

void Teleop::stopMotors()
{
  fl->setEffort(0.0);
  fr->setEffort(0.0);
  br->setEffort(0.0);
  bl->setEffort(0.0);
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

double Teleop::scale(double value)
{
  return ((value - deadzone) / (1.0 - deadzone)) * (max - min) + min;
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
    fl->setEffort(0);
    bl->setEffort(0);
  }

  if (abs(right) > deadzone)
  {
    right = scale(right);
    fr->setPoint(right);
    br->setPoint(right);
  }
  else
  {
    fr->setEffort(0);
    br->setEffort(0);
  }
}
