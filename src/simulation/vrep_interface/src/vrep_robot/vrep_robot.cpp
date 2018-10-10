#include <boost/filesystem.hpp>

#include <ros/ros.h>

#include <vrep_robot/vrep_robot.h>
#include <wheel_control/differential_drive_controller/differential_drive_controller.h>


using namespace vrep_interface;


//TODO status bar messages should probably be thrown errors
VREPRobot::VREPRobot()
{
  handle = -1;
  std::string model_file = "";
  this->wheels.initialize();

  // Load plugins
  scan_seq = 0;
}


VREPRobot::VREPRobot(std::string model_file) : VREPRobot()
{
  setModelFile(model_file);
}


simInt VREPRobot::setModelFile(std::string model_file)
{
  // Check that file exists
  if (!boost::filesystem::exists(model_file))
  {
    simAddStatusbarMessage("[method setModelFile] file doesn't exist, make sure you are specifying the full path");
    return (simInt)false;
  }
  this->model_file = model_file;
  return (simInt)true;
}


// Random placement
simInt VREPRobot::spawnRobot()
{
  simFloat *position = (simFloat*) mining_zone_centers[rand() % 2];
  simFloat rotation = mining_zone_rotations[rand() % 6];
  return spawnRobot(position, rotation);
}


// Selection placement
simInt VREPRobot::spawnRobot(simFloat *position, simFloat rotation)
{
  checkState();

  if (!loadModel())
  {
    simAddStatusbarMessage("[method spawnRobot] loadModel failed");
    return (simInt)false;
  }

  if (!move(position))
  {
    simAddStatusbarMessage("[method spawnRobot] move failed");
    return (simInt)false;
  }

  if (!rotate(rotation))
  {
    simAddStatusbarMessage("[method spawnRobot] rotate failed");
    return (simInt)false;
  }

  if (!initializeWheels()){
    simAddStatusbarMessage("[method spawnRobot] initializeWheels failed");
    return (simInt)false;
  }

  return (simInt)true;
}


simInt VREPRobot::checkState()
{
  // Reset model state if it's been deleted
  if (handle != -1)
  {
    if (simIsHandleValid(handle, -1) != 1)
    {
      handle = -1;
      simAddStatusbarMessage("[method checkState] Looks like the model was deleted, resetting model state");
      return (simInt)false;
    }
  }
  return (simInt)true;
}


simInt VREPRobot::loadModelHelper()
{
  base_link_handle = -1;
  handle = simLoadModel(model_file.c_str());
  if (handle == -1)
  {
    simAddStatusbarMessage("[method loadModel] Unable to load model");
    return (simInt)false;
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
      simAddStatusbarMessage("[method loadModel] Unable to load model (couldn't find base_link)");
      simRemoveModel(handle);
      handle = -1;
      return (simInt)false;
    }
  }
  return (simInt)true;
}


simInt VREPRobot::loadModel()
{
  if (handle == -1)
  {
    return loadModelHelper();
  }
  else
  {
    simInt success = simRemoveModel(handle);
    if (success == -1)
    {
      simAddStatusbarMessage("[method loadModel] Failed to remove previous model");
      return false;
    }
    else
    {
      return loadModelHelper();
    }
  }
}


simInt VREPRobot::move(simFloat *position)
{
  simFloat xyz[3] = {position[0], position[1], 0.0};
  if (simSetObjectPosition(handle, -1, xyz) == -1) {
    simAddStatusbarMessage("[method move] Unable to move model to position");
    return (simInt)false;
  }
  return (simInt)true;
}


simInt VREPRobot::rotate(simFloat rotation)
{
  simFloat abg[3] = {0.0, 0.0, rotation}; // Rotation is degrees from x axis
  if (simSetObjectOrientation(handle, -1, abg) == -1)
  {
    simAddStatusbarMessage("[method rotate] Unable to rotate model into orientation");
    return (simInt)false;
  }
  return true;
}


simInt VREPRobot::initializeWheels()
{
  if (wheels.updateWheelIDs() == -1) {
    simAddStatusbarMessage("[method initializeWheels] failed to initialize handles");
    simInt removedModel = simRemoveModel(handle);
    if (removedModel == -1) {
      simAddStatusbarMessage("[method initializeWheels] Also failed to remove "
                             "model, things aren't going well...");
    } else {
      handle = -1;
    }
    return (simInt)false;
  }
  return (simInt)true;
}


void VREPRobot::spinOnce()
{
  //wheel_controller->sendJointCommands();
}


void VREPRobot::setVelocity(double linear, double angular)
{
  //wheel_controller->setVelocity(linear, angular);
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



