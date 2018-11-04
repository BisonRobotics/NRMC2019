#include <vrep_plugin/imu.h>
#include <std_msgs/Header.h>
#include <Eigen/LU>
#include <Eigen/Geometry>

using namespace vrep_plugin;

using std::to_string;
using std::pair;
using std::get;
using Eigen::Matrix;

IMU::IMU(Interface *sim_interface, uint8_t id) :
  sensor_name("imu_sensor"),
  link_name("imu_mass")
{
  this->sim = sim_interface;
  this->sensor_handle = -1;
  this->link_handle = -1;

  this->nh = new ros::NodeHandle("/vrep");
  publisher = new ros::Publisher;
  (*publisher) = nh->advertise<vrep_msgs::IMU>("imu", 10, true);

  imu.header.seq = 0;
  imu.header.frame_id = "";
  imu.id = id;
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 0.0;

  initial_pass = true;
}

void IMU::initialize()
{
  sensor_handle = sim->getObjectHandle(sensor_name);
  link_handle = sim->getObjectHandle(link_name);
  mass = sim->getMass(link_handle);
  sim->info("[initialize]: Found an id of " + to_string(sensor_handle) + " for \"" + sensor_name + "\"");
  sim->info("[initialize]: Found an id of " + to_string(link_handle) + " for \"" + link_name + "\"");
  sim->info("[initialize]: IMU has a mass of " + to_string(mass) + "kg");
}

void IMU::updateState()
{
  pair<tuple3d, tuple3d> force_torque = sim->readForceSensor(sensor_handle);
  tuple3d force = get<0>(force_torque);
  Matrix<double, 4, 4> matrix = sim->getObjectMatrix(link_handle);
  ros::Time time = sim->getSimulationTime();
  double dt = (time - last_time).toSec();

  if (initial_pass || dt <= 1e-10)
  {
    initial_pass = false;
    imu.angular_velocity.x = 0.0;
    imu.angular_velocity.y = 0.0;
    imu.angular_velocity.z = 0.0;
  }
  else
  {
    Matrix<double, 4, 4> m = last_matrix.inverse() * matrix;
    Matrix<double, 3, 3> m3 = m.block<3,3>(0,0);
    Eigen::Vector3d angular_difference = m3.eulerAngles(0, 1, 2);
    // TODO find better way of checking for wrap around
    //std::cout << std::endl << "Angles1: " << angular_difference(0) << " " << angular_difference(1) << " " << angular_difference(2) << std::endl;
    for (int i = 0; i < 3; i++)
    {
      if (angular_difference(i) > M_PI / 2.0)
      {
        angular_difference(i) = angular_difference(i) - M_PI;
      }
      else if (angular_difference(i) < -M_PI / 2.0)
      {
        angular_difference(i) = angular_difference(i) + M_PI;
      }
    }
    //std::cout << "Angles2: " << angular_difference(0) << " " << angular_difference(1) << " " << angular_difference(2) << std::endl;
    imu.angular_velocity.x = angular_difference(0) / dt;
    imu.angular_velocity.y = angular_difference(1) / dt;
    imu.angular_velocity.z = angular_difference(2) / dt;
  }
  imu.header.stamp = ros::Time::now();
  imu.header.seq++;
  imu.linear_acceleration.x = -1.0 * get<0>(force) / mass;
  imu.linear_acceleration.y = -1.0 * get<1>(force) / mass;
  imu.linear_acceleration.z = get<2>(force) / mass;

  last_matrix = matrix;
  last_time = time;

  publisher->publish(imu);
}

void IMU::shutdown()
{
  publisher->shutdown();
  nh->shutdown();
  delete nh;
}

void IMU::reset()
{
  initial_pass = true;
}

