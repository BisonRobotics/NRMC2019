#include <teleop_interface/teleop_interface.h>
#include <wheel_params/wheel_params.h>



TeleopInterface::TeleopInterface(Mode mode, float max_value, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl)
{
  this->fl = fl;
  this->fr = fr;
  this->br = br;
  this->bl = bl;
  stopMotors();
  setMode(mode);
  setMax(max_value);
  this->internally_alloc = false;
}

TeleopInterface::TeleopInterface(Mode mode, float velocity)
{
  this->fl = new VescAccess(front_left_param);
  this->fr = new VescAccess(front_right_param);
  this->br = new VescAccess(back_right_param);
  this->bl = new VescAccess(back_left_param);
  stopMotors();
  setMode(mode);
  setMax(velocity);
  this->internally_alloc = true;
}

void TeleopInterface::setMax(float max_value)
{
  this->max_value = std::abs(max_value);
}

float TeleopInterface::getMax()
{
  return this->max_value;
}

void TeleopInterface::setMode(TeleopInterface::Mode mode)
{
  this->mode = mode;
}

TeleopInterface::Mode TeleopInterface::getMode()
{
  return mode;
}

void TeleopInterface::stopMotors()
{
  fl->setLinearVelocity(0.0f);
  fr->setLinearVelocity(0.0f);
  br->setLinearVelocity(0.0f);
  bl->setLinearVelocity(0.0f);
}

float TeleopInterface::clamp(float number, float max, float min)
{
  return std::max(min, std::min(number, max));
}

void TeleopInterface::update(float left, float right)
{
  if (std::abs(left) > 0.001f)
  {
    if (mode == velocity)
    {
      fl->setLinearVelocity(left * max_value);
      bl->setLinearVelocity(left * max_value);
    }
    else if (mode == duty)
    {
      fl->setDuty(clamp(left*max_value, 0.95f, -0.95f));
      bl->setDuty(clamp(left*max_value, 0.95f, -0.95f));
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
      fr->setLinearVelocity(right * max_value);
      br->setLinearVelocity(right * max_value);
    }
    else if (mode == duty)
    {
      fr->setDuty(clamp(left*max_value, 0.95f, -0.95f));
      br->setDuty(clamp(left*max_value, 0.95f, -0.95f));
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

TeleopInterface::~TeleopInterface()
{
  if (internally_alloc)
  {
    delete fl;
    delete fr;
    delete bl;
    delete br;
  }
}
