#ifndef SENSOR_ACCESS_VREP_IMU_H
#define SENSOR_ACCESS_VREP_IMU_H

#include "sensor_access/imu_sensor_interface.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include <vrep_msgs/IMU.h>

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class VrepImu : public ImuSensorInterface
{
  public:
    VrepImu(double accelerometer_offset, double accelerometer_deviation,
            double gyroscope_offset,     double gyroscope_deviation);

    double getX() override;
    double getY() override;
    double getOmega() override;
    ReadableSensors::ReadStatus receiveData() override;
    tf2::Quaternion getOrientation() override;

    void updateNoiseModel(double accelerometer_offset, double accelerometer_deviation,
                          double gyroscope_offset,     double gyroscope_deviation);

  protected:
    std::default_random_engine generator;
    std::normal_distribution<double> accelerometer_distribution, gyroscope_distribution;

  private:
    void imuCallback(const vrep_msgs::IMU::ConstPtr &msg);

    double x_acc;
    double y_acc;
    double omega;
    bool is_data_valid;
    ros::Subscriber subscriber;
    ros::NodeHandle nh;
    tf2::Quaternion orientation;
};

#endif
