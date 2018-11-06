#include <vrep_access/vrep_position_sensor.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2/convert.h"
#include "std_msgs/Bool.h"

VrepPositionSensor::VrepPositionSensor(double angular_offset, double angular_deviation,
                                       double linear_offset,  double linear_deviation) :
  nh()
{
  subscriber = nh.subscribe("/vrep_access/pose", 1, &VrepPositionSensor::callback, this);
  x = 0.0;
  y = 0.0;
  z = 0.0;
  theta = 0.0;
  data_is_valid = false;
  updateNoiseModel(angular_offset, angular_deviation, linear_offset, linear_deviation);
}

double VrepPositionSensor::getX()
{
  return x;
}

double VrepPositionSensor::getY()
{
  return y;
}

double VrepPositionSensor::getZ ()
{
  return z;
}

double VrepPositionSensor::getTheta()
{
  return theta;
}

ReadableSensors::ReadStatus VrepPositionSensor::receiveData()
{
  if (data_is_valid)
  {
    return ReadableSensors::ReadStatus::READ_SUCCESS;
  }
  return ReadableSensors::ReadStatus::READ_FAILED;
}

void VrepPositionSensor::callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  this->x = msg->pose.position.x + linear_distribution(generator);
  this->y = msg->pose.position.y + linear_distribution(generator);
  this->z = msg->pose.position.z + linear_distribution(generator);
  this->theta = qtToTheta(msg->pose.orientation) + angular_distribution(generator);
  data_is_valid = true;
}

bool VrepPositionSensor::isFloating()
{
  return false;
}

double VrepPositionSensor::qtToTheta(geometry_msgs::Quaternion qt)
{
  tf2::Matrix3x3 m;
  m.setRotation(tf2::Quaternion(qt.x, qt.y, qt.z, qt.w));
  double tmp1, tmp2, theta;
  m.getRPY(tmp1, tmp2, theta);
  return theta;
}

void VrepPositionSensor::updateNoiseModel(double angular_offset, double angular_deviation,
                                          double linear_offset,  double linear_deviation)
{
  angular_distribution = std::normal_distribution<double>(angular_offset, angular_deviation);
  linear_distribution = std::normal_distribution<double>(linear_offset, linear_deviation);
}
