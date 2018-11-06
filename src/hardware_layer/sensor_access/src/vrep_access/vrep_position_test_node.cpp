#include <ros/ros.h>
#include <vrep_access/vrep_position_sensor.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "vrep_imu_test_node");
  ros::start();
  ros::Rate rate(10);

  PosSensorInterface *sensor = new VrepPositionSensor(0.0, 0.05, 0.0, 0.05);

  while (ros::ok())
  {
    ros::spinOnce();
    std::cout << sensor->getX() << " " << sensor->getY() << " " << sensor->getZ() << " " << sensor->getTheta() << std::endl;
    rate.sleep();
  }

  return 0;
}