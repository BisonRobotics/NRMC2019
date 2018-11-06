#include <ros/ros.h>
#include <vrep_access/vrep_imu.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "vrep_imu_test_node");
  ros::start();
  ros::Rate rate(10);

  ImuSensorInterface *imu = new VrepImu(0.0, 0.001, 0.0, 0.001);

  while (ros::ok())
  {
    ros::spinOnce();
    std::cout << imu->getX() << " " << imu->getY() << " " << imu->getOmega() << std::endl;
    rate.sleep();
  }

  return 0;
}