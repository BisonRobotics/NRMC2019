#include <vrep_drivers/vrep_driver_base.h>
#include <driver_access/mode.h>
#include <driver_access/limits.h>
#include <vrep_interface/sim_interface.h>
#include <vrep_library/v_repConst.h>

using namespace vrep_interface;

using std::string;
using std::to_string;
using vrep_msgs::VREPDriverMessage;
using vrep_msgs::VREPDriverMessageConstPtr;
using driver_access::Limits;
using driver_access::Mode;
using driver_access::ID;
using driver_access::name;

VREPDriverBase::VREPDriverBase(SimInterface *sim_interface, ID id) :
    sim(sim_interface),
    DriverAccess(Limits(-1e10, 1e10, 0, 1e10, 0, 1e10), id),
    handle(-1),
    joint_name(name(id) + "_joint")
{
  nh = new ros::NodeHandle("/vrep/" + name(id));
  subscriber = new ros::Subscriber;
  publisher = new ros::Publisher;
  pid_set_server = new ros::ServiceServer;
  pid_get_server = new ros::ServiceServer;

  (*subscriber) = nh->subscribe("command", 10, &VREPDriverBase::callback, this);
  (*publisher) = nh->advertise<VREPDriverMessage>("state", 10, true);
  (*pid_set_server) = nh->advertiseService("set_pid", &VREPDriverBase::setPIDCallback, this);
  (*pid_get_server) = nh->advertiseService("get_pid", &VREPDriverBase::getPIDCallback, this);


  state.id = static_cast<uint8_t>(id);
  command.mode = static_cast<uint8_t>(Mode::velocity);
  command.header.seq = 0;
  command.position = 0;
  command.velocity = 0;
  command.effort = 0;

}

double VREPDriverBase::getVelocity()
{
  return command.velocity;
}

double VREPDriverBase::getEffort()
{
  return command.effort;
}

double VREPDriverBase::getPosition()
{
  return command.position;
}

Mode VREPDriverBase::getMode()
{
  return static_cast<Mode>(command.mode);
}

double VREPDriverBase::setPoint()
{
  Mode mode = getMode();
  if (mode == Mode::position)
  {
    setPosition(getPosition());
  }
  else if (mode == Mode::velocity)
  {
    setVelocity(getVelocity());
  }
  else if (mode == Mode::effort)
  {
    setEffort(getEffort());
  }
  else
  {
    throw std::runtime_error("[setPoint]: Invalid mode selected");
  }
}

void VREPDriverBase::callback(const vrep_msgs::VREPDriverMessageConstPtr &message)
{
  command.header.seq = message->header.seq;
  command.position = message->position;
  command.velocity = message->velocity;
  command.effort = message->effort;
  command.id = message->id;
  command.mode = message->mode;
}

void VREPDriverBase::updateHeader(std_msgs::Header *header)
{
  header->stamp = ros::Time::now();
  header->seq = seq++;
}

void VREPDriverBase::updateHandle()
{
  handle = sim->getObjectHandle(joint_name);
  sim->info("[updateHandle]: Found an id of " + to_string(handle) + " for \"" + joint_name + "\"");
}

void VREPDriverBase::shutdown()
{
  subscriber->shutdown();
  publisher->shutdown();
  nh->shutdown();
  delete subscriber;
  delete publisher;
  delete nh;
}

bool VREPDriverBase::getPIDCallback(vrep_msgs::PIDGetRequest &req, vrep_msgs::PIDGetResponse &res)
{
  try
  {
    res.p = sim->getFloatParameter(handle, sim_jointfloatparam_pid_p);
    res.i = sim->getFloatParameter(handle, sim_jointfloatparam_pid_i);
    res.d = sim->getFloatParameter(handle, sim_jointfloatparam_pid_d);
    res.success = 1;
    sim->info("[getPIDCallback]: " + joint_name + " PID = ("
              + std::to_string(res.p) + ", "
              + std::to_string(res.i) + ", "
              + std::to_string(res.d) + ")");
    return true;
  }
  catch (std::runtime_error &e)
  {
    sim->info(e.what());
    res.success = 0;
    return false;
  }
}

bool VREPDriverBase::setPIDCallback(vrep_msgs::PIDSetRequest &req, vrep_msgs::PIDSetResponse &res)
{
  try
  {
    sim->setParameter(handle, sim_jointfloatparam_pid_p, req.p);
    sim->setParameter(handle, sim_jointfloatparam_pid_i, req.i);
    sim->setParameter(handle, sim_jointfloatparam_pid_d, req.d);
    res.success = 1;
    sim->info("[setPIDCallback]: " + joint_name + " PID = ("
                                   + std::to_string(req.p) + ", "
                                   + std::to_string(req.i) + ", "
                                   + std::to_string(req.d) + ")");
    return true;
  }
  catch (std::runtime_error &e)
  {
    sim->info(e.what());
    res.success = 0;
    return false;
  }
}


