#ifndef SENSOR_ACCESS_VREP_POSITION_SENSOR_H
#define SENSOR_ACCESS_VREP_POSITION_SENSOR_H

#include <sensor_access/pos_sensor_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <random>

class VrepPositionSensor : public PosSensorInterface
{
  public:
    explicit VrepPositionSensor(double angular_offset, double angular_deviation,
                                double linear_offset,  double linear_deviation);
    double getX() override;
    double getY() override;
    double getZ () override;
    double getTheta() override;
    bool isFloating() override;
    ReadableSensors::ReadStatus receiveData() override;

    void updateNoiseModel(double angular_offset, double angular_deviation,
                          double linear_offset,  double linear_deviation);

  protected:
    std::default_random_engine generator;
    std::normal_distribution<double> angular_distribution, linear_distribution;

  private:
    double qtToTheta(geometry_msgs::Quaternion);
    void callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    ros::Subscriber subscriber;
    ros::NodeHandle nh;
    bool data_is_valid;
    double x, y, z, theta;
};

#endif  // SENSOR_ACCESS_VREP_POSITION_SENSOR_H
