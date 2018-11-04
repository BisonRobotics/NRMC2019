#include <vrep_imu/vrep_imu.h>

VrepImu::VrepImu() : nh_(), orientation(0,0,0,1)
{
  this->subscriber = this->nh_.subscribe("/vrep/imu", 1, &VrepImu::imuCallback, this);
  x_acc = 0.0;
  y_acc = 0.0;
  omega = 0.0;
  is_data_valid = false;
}

double VrepImu::getX(void)
{
  return x_acc;
}

double VrepImu::getY(void)
{
  return y_acc;
}

double VrepImu::getOmega(void)
{
  return omega;
}

ReadableSensors::ReadStatus VrepImu::receiveData()
{
  if (is_data_valid)
  {
    return ReadableSensors::ReadStatus::READ_SUCCESS;
  }
  else
  {
    return ReadableSensors::ReadStatus::READ_FAILED;
  }
}

void VrepImu::imuCallback(const vrep_msgs::IMU::ConstPtr &msg)
{
  is_data_valid = true;
  x_acc = msg->linear_acceleration.x;
  y_acc = msg->linear_acceleration.y;
  omega = msg->angular_velocity.z;
}

tf2::Quaternion VrepImu::getOrientation()
{
  return orientation;
}
