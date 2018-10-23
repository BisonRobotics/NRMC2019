#include <gazebo_access_server/gazebo_access_server.h>
#include <driver_access/mode.h>
#include <driver_access/limits.h>

using namespace gazebo;

using std::string;
using std::to_string;
using transport::NodePtr;
using transport::Node;
using driver_access::Limits;
using driver_access::Mode;
using driver_access::ID;
using driver_access::name;

GazeboDriver::GazeboDriver(physics::ModelPtr model, ID id) :
    DriverAccess(Limits(-1e10, 1e10, 0, 1e10, 0, 1e10), id),
    joint_name(name(id) + "_joint")
{
  this->model = model;
  joint = model->GetJoint(joint_name);
  pid = common::PID(1.0, 0.0, 0.0);

  node = NodePtr(new Node);
  node->Init(model->GetWorld()->Name());
  subscriber = node->Subscribe("~/" + model->GetName() + "/cmd", &GazeboDriver::cmd_callback, this);

  model->GetJointController()->SetVelocityPID(joint->GetScopedName(), pid);
  model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), 0.0);
}

double GazeboDriver::getVelocity()
{
  return joint->GetVelocity(0);
}

double GazeboDriver::getEffort()
{
  return joint->GetForce(0);
}

double GazeboDriver::getPosition()
{
  return 0.0;
}

void GazeboDriver::setDriverPosition(double position)
{
  //setJointPosition(handle, position);
}

void GazeboDriver::setDriverVelocity(double velocity)
{
  model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), velocity / 0.1524);
}

void GazeboDriver::setDriverEffort(double effort)
{
}


Mode GazeboDriver::getMode()
{
  //return static_cast<Mode>(command.mode);
}

double GazeboDriver::setPoint()
{
  Mode mode = getMode();
  if (mode == Mode::position)
  {
    //setPosition(command.position);
  }
  else if (mode == Mode::velocity)
  {
    //setVelocity(command.velocity);
  }
  else if (mode == Mode::effort)
  {
    //setEffort(command.effort);
  }
  else
  {
    throw std::runtime_error("[setPoint]: Invalid mode selected");
  }
}

void GazeboDriver::cmd_callback(ConstVector3dPtr &message)
{
  setVelocity(message->x());
}

