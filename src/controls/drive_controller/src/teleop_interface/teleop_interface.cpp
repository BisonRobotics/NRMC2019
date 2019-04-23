#include <teleop_interface/teleop_interface.h>
#include <boost/algorithm/clamp.hpp>


using boost::algorithm::clamp;


TeleopInterface::TeleopInterface(Mode mode, float max_value, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl)
{
  this->fl = fl;
  this->fr = fr;
  this->br = br;
  this->bl = bl;
  stopMotors();
  setMode(mode);
  setMax(max_value);
}

void TeleopInterface::setMax(float max_value)
{
  if (mode == velocity)
  {
    this->max_value = std::abs(max_value);
  }
  else if (mode == duty)
  {
    this->max_value = std::abs(clamp(max_value, -0.95f, 0.95f));
  }
  else
  {
    this->max_value = 0.0f;
  }
}

float TeleopInterface::getMax()
{
  return this->max_value;
}

void TeleopInterface::setMode(TeleopInterface::Mode mode)
{
  this->mode = mode;
  if (mode == duty)
  {
    this->max_value = std::abs(clamp(max_value, -0.95f, 0.95f));
  }
}

TeleopInterface::Mode TeleopInterface::getMode()
{
  return mode;
}

void TeleopInterface::stopMotors()
{
  fl->setTorque(0.0f);
  fr->setTorque(0.0f);
  br->setTorque(0.0f);
  bl->setTorque(0.0f);
}

void TeleopInterface::update(float left, float right)
{
  if (std::abs(left) > 0.001f)
  {
    if (mode == velocity)
    {
      fl->setLinearVelocity(clamp(left*max_value, -max_value, max_value));
      bl->setLinearVelocity(clamp(left*max_value, -max_value, max_value));
    }
    else if (mode == duty)
    {
      fl->setDuty(clamp(left*max_value, -0.95f, 0.95f));
      bl->setDuty(clamp(left*max_value, -0.95f, 0.95f));
    }
    else
    {
      fl->setTorque(0);
      bl->setTorque(0);
    }
  }
  else
  {
    fl->setTorque(0);
    bl->setTorque(0);
  }

  if (std::abs(right) > 0.001f)
  {
    if (mode == velocity)
    {
      fr->setLinearVelocity(clamp(right*max_value, -max_value, max_value));
      br->setLinearVelocity(clamp(right*max_value, -max_value, max_value));
    }
    else if (mode == duty)
    {
      fr->setDuty(clamp(right*max_value, -0.95f, 0.95f));
      br->setDuty(clamp(right*max_value, -0.95f, 0.95f));
    }
    else
    {
      fr->setTorque(0);
      br->setTorque(0);
    }
  }
  else
  {
    fr->setTorque(0);
    br->setTorque(0);
  }

  fl->getTorque();
  bl->getTorque();
  br->getTorque();
  fr->getTorque();
}
