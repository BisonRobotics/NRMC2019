#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <vrep_plugin/robot.h>
#include <driver_access/params.h>
#include <vrep_plugin/interface.h>
#include <vrep_plugin/exceptions.h>

using namespace vrep_plugin;

using driver_access::ID;
using std::to_string;
using std::get;

Robot::Robot(Interface *sim) :
    sim(sim), handle(-1), model_file(""), imu(sim, 1),
    fl(sim, ID::front_left_wheel), fr(sim, ID::front_right_wheel),
    br(sim, ID::back_right_wheel), bl(sim, ID::back_left_wheel) {}

void Robot::initialize(std::string model_file)
{
  if (!boost::filesystem::exists(model_file))
  {
    throw std::runtime_error("[method setModelFile] file doesn't exist, make sure you are specifying the full path");
  }
  this->model_file = model_file;
}

void Robot::spawnRobot()
{
  double y = mining_zone_centers[rand() % 2];
  double rotation = mining_zone_rotations[rand() % 6];
  spawnRobot(0.75f, y, rotation);
}

void Robot::spawnRobot(double x, double y, double rotation)
{
  checkState();
  loadModel();
  sim->setObjectPosition(handle, -1, x, y, 0);
  sim->setObjectOrientation(handle, -1, 0, 0, rotation);
  updateHandles();
}

void Robot::checkState()
{
  if (handle != -1)
  {
    if (!sim->isHandleValid(handle, -1))
    {
      handle = -1;
      throw std::runtime_error("[checkState]: Looks like the model was deleted, resetting model state");
    }
  }
}

void Robot::loadModelHelper()
{
  base_link_handle = -1;
  handle = sim->loadModel(model_file);
  try
  {
    base_link_handle = sim->findObjectInTree(handle, "base_link", sim_object_dummy_type);
    sim->info("[loadModel]: Found an id of " + to_string(base_link_handle) + " for base_link_handle");
    sim->info("[loadModel]: Found an id of " + to_string(handle) + " for handle");
  }
  catch (vrep_error &error)
  {
    sim->removeModel(handle);
    handle = -1;
    throw error;
  }
}

void Robot::loadModel()
{
  if (handle == -1)
  {
    loadModelHelper();
  }
  else
  {
    sim->removeModel(handle);
    loadModelHelper();
  }
}

void Robot::updateHandles()
{
  fl.initialize();
  fr.initialize();
  br.initialize();
  bl.initialize();
  imu.initialize();
}

void Robot::getPose(geometry_msgs::Pose *pose)
{
  tuple3d origin = sim->getObjectPosition(base_link_handle, -1);
  tuple3d angles = sim->getObjectOrientation(base_link_handle, -1);

  tf::Quaternion rotation;
  rotation.setEuler(get<0>(angles), get<1>(angles), get<2>(angles));

  pose->position.x = get<0>(origin);
  pose->position.y = get<1>(origin);
  pose->position.z = get<2>(origin);
  pose->orientation.x = rotation.x();
  pose->orientation.y = rotation.y();
  pose->orientation.z = rotation.z();
  pose->orientation.w = rotation.w();
}

void Robot::spinOnce()
{
  fl.updateState();
  fr.updateState();
  br.updateState();
  bl.updateState();
  imu.updateState();
  fl.setPoint();
  fr.setPoint();
  br.setPoint();
  bl.setPoint();
  // TODO update robot state too
}

void Robot::shutdown()
{
  fl.shutdown();
  fr.shutdown();
  br.shutdown();
  bl.shutdown();
}

void Robot::reset()
{
  imu.reset();
}



