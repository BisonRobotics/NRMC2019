#include <vrep_access/vrep_imu.h>

VrepImu::VrepImu(double accelerometer_offset, double accelerometer_deviation,
                 double gyroscope_offset,     double gyroscope_deviation) :
  nh(),
  orientation(0,0,0,1)
{
  this->subscriber = this->nh.subscribe("/vrep/imu", 1, &VrepImu::imuCallback, this);
  x_acc = 0.0;
  y_acc = 0.0;
  omega = 0.0;
  is_data_valid = false;
  updateNoiseModel(accelerometer_offset, accelerometer_deviation, gyroscope_offset, gyroscope_deviation);
}

double VrepImu::getX()
{
  return x_acc;
}

double VrepImu::getY()
{
  return y_acc;
}

double VrepImu::getOmega()
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
  x_acc = msg->linear_acceleration.x + accelerometer_distribution(generator);
  y_acc = msg->linear_acceleration.y + accelerometer_distribution(generator);
  omega = msg->angular_velocity.z + gyroscope_distribution(generator);
}

tf2::Quaternion VrepImu::getOrientation()
{
  return orientation;
}

void VrepImu::updateNoiseModel(double accelerometer_offset, double accelerometer_deviation,
                               double gyroscope_offset,     double gyroscope_deviation)
{
  accelerometer_distribution = std::normal_distribution<double>(accelerometer_offset, accelerometer_deviation);
  gyroscope_distribution = std::normal_distribution<double>(gyroscope_offset, gyroscope_deviation);
}
