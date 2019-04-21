#include <dig_control/dig_controller_sim.h>
#include <fstream>
#include <sstream>
#include <vector>

using namespace dig_control;

DigControllerSim::DigControllerSim(std::string file) :
  i(0), increment(false), control_state(ControlState::ready)
{
  std::ifstream data(file);
  std::string line;
  while(std::getline(data,line))
  {
    std::stringstream lineStream(line);
    std::string cell;
    std::getline(lineStream, cell,','); bucket_duty.emplace_back(std::stof(cell));
    std::getline(lineStream, cell,','); backhoe_duty.emplace_back(std::stof(cell));
    std::getline(lineStream, cell,','); central_duty.emplace_back(std::stof(cell));
    std::getline(lineStream, cell,','); vibrator_duty.emplace_back(std::stof(cell));
    std::getline(lineStream, cell,','); bucket_state.emplace_back((BucketState)std::stoi(cell));
    std::getline(lineStream, cell,','); backhoe_state.emplace_back((BackhoeState)std::stoi(cell));
    std::getline(lineStream, cell,','); central_drive_state.emplace_back((CentralDriveState)std::stoi(cell));
    std::getline(lineStream, cell,','); dig_state.emplace_back((DigState)std::stoi(cell));
    std::getline(lineStream, cell,','); goal_state.emplace_back((ControlState)std::stoi(cell));
    std::getline(lineStream, cell,','); central_position.emplace_back(std::stoi(cell));
  }
  size = (int)bucket_duty.size();
  printf("Size: %i", size);
}

void DigControllerSim::update()
{
  if (i >= size) i = 0;
}

void DigControllerSim::setControlState(ControlState goal)
{
  switch (goal)
  {
    case ControlState::dig:
    {
      i = 0;
      increment = true;
      control_state = ControlState::dig;
      break;
    }
    case ControlState::finish_dig:
    {
      i = 0;
      increment = false;
      control_state = ControlState::ready;
      break;
    }
    case ControlState::manual:
    {
      i = 0;
      increment = false;
      control_state = ControlState::manual;
      break;
    }
    default:
    {
      i = 0;
      increment = false;
      control_state = ControlState::ready;
      break;
    }
  }
}

void DigControllerSim::setCentralDriveDuty(float value){}
void DigControllerSim::setBackhoeDuty(float value){}
void DigControllerSim::setBucketDuty(float value){}
void DigControllerSim::setVibratorDuty(float value){}
void DigControllerSim::stop(){}

ControlState DigControllerSim::getControlState() const
{
  return goal_state[i];
}

CentralDriveState DigControllerSim::getCentralDriveState() const
{
  return central_drive_state[i];
}

BackhoeState DigControllerSim::getBackhoeState() const
{
  return backhoe_state[i];
}

BucketState DigControllerSim::getBucketState() const
{
  return bucket_state[i];
}

DigState DigControllerSim::getDigState() const
{
  return dig_state[i];
}

float DigControllerSim::getCentralDriveDuty() const
{
  return central_duty[i];
}

float DigControllerSim::getBackhoeDuty() const
{
  return backhoe_duty[i];
}

float DigControllerSim::getBucketDuty() const
{
  return bucket_duty[i];
}

float DigControllerSim::getVibratorDuty() const
{
  return vibrator_duty[i];
}

int DigControllerSim::getCentralDrivePosition() const
{
  return central_position[i];
}

int DigControllerSim::getBackhoePosition() const
{
  return 0;
}

std::string DigControllerSim::getControlStateString() const
{
  return to_string(goal_state[i]);
}

std::string DigControllerSim::getCentralDriveStateString() const
{
  return to_string(central_drive_state[i]);
}

std::string DigControllerSim::getBackhoeStateString() const
{
  return to_string(backhoe_state[i]);
}

std::string DigControllerSim::getDigStateString() const
{
  return to_string(dig_state[i]);
}

std::string DigControllerSim::getBucketStateString() const
{
  return to_string(bucket_state[i]);
}
