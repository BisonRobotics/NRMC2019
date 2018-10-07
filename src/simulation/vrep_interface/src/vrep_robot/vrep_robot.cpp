#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <vrep_robot/vrep_robot.h>
#include <driver_access/params.h>
#include <vrep_interface/vrep_server.h>

using namespace vrep_interface;

using driver_access::ID;
using std::to_string;

//TODO status bar messages should probably be thrown errors
VREPRobot::VREPRobot() : handle(-1), model_file(""),
  fl(ID::front_left_wheel), fr(ID::front_right_wheel),
  br(ID::back_right_wheel), bl(ID::back_left_wheel){}

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
  simFloat y = mining_zone_centers[rand() % 2];
  simFloat rotation = mining_zone_rotations[rand() % 6];
  spawnRobot(0.75f, y, rotation);
}

void VREPRobot::spawnRobot(simFloat x, simFloat y, simFloat rotation)
{
  checkState();
  loadModel();
  move(x, y);
  rotate(rotation);
  updateWheelHandles();
}

void VREPRobot::checkState()
{
  // Reset model state if it's been deleted
  if (handle != -1)
  {
    if (simIsHandleValid(handle, -1) != 1)
    {
      handle = -1;
      throw std::runtime_error("[checkState]: Looks like the model was deleted, resetting model state");
    }
  }
}

void VREPRobot::loadModelHelper()
{
  base_link_handle = -1;
  handle = simLoadModel(model_file.c_str());
  if (handle == -1)
  {
    throw std::runtime_error("[loadModel]: Unable to load model");
  }
  else
  {
    simInt *tree_handles;
    simInt handle_count = -1;
    tree_handles = simGetObjectsInTree(handle, sim_object_dummy_type, 0x00, &handle_count);
    for (int i = 0; i < handle_count; i++)
    {
      simChar *name_c = simGetObjectName(tree_handles[i]);
      if (name_c != NULL)
      {
        std::string name = std::string(name_c);
        if (name == "base_link")
        {
          base_link_handle = tree_handles[i];
          VREPServer::info("[loadModel]: Found an id of " + to_string(base_link_handle) + " for base_link_handle");
        }
      }
    }
    // TODO check for other buffers that need to be released
    simReleaseBuffer((simChar*)tree_handles);
    if (base_link_handle == -1)
    {
      simRemoveModel(handle);
      handle = -1;
      throw std::runtime_error("[loadModel]: Unable to load model (couldn't find base_link)");
    }
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
    if (simRemoveModel(handle) == -1)
    {
      throw std::runtime_error("[loadModel]: Failed to remove previous model");
    }
    else
    {
      loadModelHelper();
    }
  }
}

void VREPRobot::move(simFloat x, simFloat y)
{
  simFloat xyz[3] = {x, y, 0.0};
  if (simSetObjectPosition(handle, -1, xyz) == -1) {
    throw std::runtime_error("[move]: Unable to move model to position");
  }
}

void VREPRobot::rotate(simFloat rotation)
{
  simFloat abg[3] = {0.0, 0.0, rotation}; // Rotation is degrees from x axis
  if (simSetObjectOrientation(handle, -1, abg) == -1)
  {
    throw std::runtime_error("[rotate]: Unable to rotate model into orientation");
  }
}

void VREPRobot::updateWheelHandles()
{
  fl.updateHandle();
  fr.updateHandle();
  br.updateHandle();
  bl.updateHandle();
}

void VREPRobot::getPosition(tf::Transform *position)
{
  simFloat origin[3];
  simFloat euler_angles[3];
  simGetObjectPosition(base_link_handle, -1, origin);
  simGetObjectOrientation(base_link_handle, -1, euler_angles);

  tf::Quaternion rotation;
  rotation.setEuler(euler_angles[0], euler_angles[1], euler_angles[2]);

  position->setOrigin(tf::Vector3(origin[0], origin[1], origin[2]));
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



