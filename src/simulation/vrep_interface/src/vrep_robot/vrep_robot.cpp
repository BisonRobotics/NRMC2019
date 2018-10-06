#include <boost/filesystem.hpp>

#include <ros/ros.h>

#include <vrep_robot/vrep_robot.h>


using namespace vrep_interface;


//TODO status bar messages should probably be thrown errors
VREPRobot::VREPRobot()
{
  handle = -1;
  std::string model_file = "";
}

void VREPRobot::initialize(std::string model_file)
{
  if (!boost::filesystem::exists(model_file))
  {
    std::runtime_error("[method setModelFile] file doesn't exist, make sure you are specifying the full path");
  }
  this->model_file = model_file;

  wheels.emplace_back(0, "wheel_front_left_joint");
  wheels.emplace_back(1, "wheel_front_right_joint");
  wheels.emplace_back(2, "wheel_back_left_joint");
  wheels.emplace_back(3, "wheel_back_left_joint");
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
      throw std::runtime_error("[method checkState] Looks like the model was deleted, resetting model state");
    }
  }
}

void VREPRobot::loadModelHelper()
{
  base_link_handle = -1;
  handle = simLoadModel(model_file.c_str());
  if (handle == -1)
  {
    throw std::runtime_error("[method loadModel] Unable to load model");
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
          simAddStatusbarMessage(("[method loadModel] Found: " + name).c_str());
          base_link_handle = tree_handles[i];
          simAddStatusbarMessage(("[method loadModel] Found an id of " + std::to_string(base_link_handle)
                                 + " for base_link_handle").c_str());
        }
      }
    }
    // TODO check for other buffers that need to be released
    simReleaseBuffer((simChar*)tree_handles);
    if (base_link_handle == -1)
    {
      simRemoveModel(handle);
      handle = -1;
      throw std::runtime_error("[method loadModel] Unable to load model (couldn't find base_link)");
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
      throw std::runtime_error("[method loadModel] Failed to remove previous model");
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
    throw std::runtime_error("[method move] Unable to move model to position");
  }
}

void VREPRobot::rotate(simFloat rotation)
{
  simFloat abg[3] = {0.0, 0.0, rotation}; // Rotation is degrees from x axis
  if (simSetObjectOrientation(handle, -1, abg) == -1)
  {
    throw std::runtime_error("[method rotate] Unable to rotate model into orientation");
  }
}

void VREPRobot::updateWheelHandles()
{
  for (int i = 0; i < wheels.size(); i++)
  {
    wheels[i].updateHandle();
  }
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
  for (int i = 0; i < wheels.size(); i++)
  {
    wheels[i].updateState();
  }
  // TODO update robot state too
}

void VREPRobot::shutdown()
{
  for (int i = 0; i < wheels.size(); i++)
  {
    wheels[i].shutdown();
  }
}



