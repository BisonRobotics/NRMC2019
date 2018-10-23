#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <vrep_robot/vrep_robot.h>
#include <driver_access/params.h>
#include <vrep_interface/sim_interface.h>
#include <vrep_exceptions/vrep_exceptions.h>

using namespace vrep_interface;

using driver_access::ID;
using std::to_string;
using std::get;

//TODO status bar messages should probably be thrown errors
VREPRobot::VREPRobot(SimInterface *sim) :
    sim(sim), handle(-1), model_file(""),
    fl(sim, ID::front_left_wheel), fr(sim, ID::front_right_wheel),
    br(sim, ID::back_right_wheel), bl(sim, ID::back_left_wheel) {}

void VREPRobot::initialize(std::string model_file)
{
  if (!boost::filesystem::exists(model_file))
  {
    throw std::runtime_error("[method setModelFile] file doesn't exist, make sure you are specifying the full path");
  }
  this->model_file = model_file;
}

void VREPRobot::spawnRobot()
{
  simFloat y = (simFloat) mining_zone_centers[rand() % 2];
  simFloat rotation = (simFloat) mining_zone_rotations[rand() % 6];
  spawnRobot(0.75f, y, rotation);
}

void VREPRobot::spawnRobot(double x, double y, double rotation)
{
  checkState();
  loadModel();
  sim->setObjectPosition(handle, -1, x, y, 0);
  sim->setObjectOrientation(handle, -1, 0, 0, rotation);
  updateWheelHandles();
}

void VREPRobot::checkState()
{
  // Reset model state if it's been deleted
  if (handle != -1)
  {
    if (!sim->isHandleValid(handle, -1))
    {
      handle = -1;
      throw std::runtime_error("[checkState]: Looks like the model was deleted, resetting model state");
    }
  }
}

void VREPRobot::loadModelHelper()
{
  base_link_handle = -1;
  handle = sim->loadModel(model_file);

  try
  {
    base_link_handle = sim->findObjectInTree(handle, "base_link", sim_object_dummy_type);
    std::cout << "loadModelHelper" << std::endl;
    sim->info("[loadModel]: Found an id of " + to_string(base_link_handle) + " for base_link_handle");
    sim->info("[loadModel]: Found an id of " + to_string(handle) + " for handle");
    std::cout << "loadModelHelper" << std::endl;
  }
  catch (vrep_error &error)
  {
    sim->removeModel(handle);
    handle = -1;
    throw error;
  }
}

void VREPRobot::loadModel()
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

void VREPRobot::updateWheelHandles()
{
  fl.initialize();
  fr.initialize();
  br.initialize();
  bl.initialize();
}

void VREPRobot::getPosition(tf::Transform *position)
{
  tuple3d origin = sim->getObjectPosition(base_link_handle, -1);
  tuple3d angles = sim->getObjectOrientation(base_link_handle, -1);

  tf::Quaternion rotation;
  rotation.setEuler(get<0>(angles), get<1>(angles), get<2>(angles));

  position->setOrigin(tf::Vector3(get<0>(origin), get<1>(origin), get<2>(origin)));
  position->setRotation(rotation);
}

void VREPRobot::spinOnce()
{
  fl.updateState();
  fr.updateState();
  br.updateState();
  bl.updateState();
  fl.setPoint();
  fr.setPoint();
  br.setPoint();
  bl.setPoint();
  // TODO update robot state too
}

void VREPRobot::shutdown()
{
  fl.shutdown();
  fr.shutdown();
  br.shutdown();
  bl.shutdown();
}



