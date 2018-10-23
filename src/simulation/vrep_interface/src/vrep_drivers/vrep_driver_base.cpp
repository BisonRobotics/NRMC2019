#include <vrep_drivers/vrep_driver_base.h>
#include <driver_access/mode.h>
#include <driver_access/limits.h>
#include <vrep_interface/sim_interface.h>
#include <vrep_library/v_repConst.h>

using namespace vrep_interface;

using std::get;
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
    joint_handle(-1),
    link_handle(-1),
    joint_name(name(id) + "_joint"),
    link_name(name(id))
{
  nh = new ros::NodeHandle("/vrep/" + link_name);
  subscriber = new ros::Subscriber;
  publisher = new ros::Publisher;
  params_server = new ros::ServiceServer;

  (*subscriber) = nh->subscribe("command", 10, &VREPDriverBase::commandCallback, this);
  (*publisher) = nh->advertise<VREPDriverMessage>("state", 10, true);

  state.id = static_cast<uint8_t>(id);
  command.mode = static_cast<uint8_t>(Mode::velocity);
  command.header.seq = 0;
  command.position = 0;
  command.velocity = 0;
  command.effort = 0;
}

double VREPDriverBase::getVelocity()
{
  return state.velocity;
}

double VREPDriverBase::getEffort()
{
  return state.effort;
}

double VREPDriverBase::getPosition()
{
  return state.position;
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
    setPosition(command.position);
  }
  else if (mode == Mode::velocity)
  {
    setVelocity(command.velocity);
  }
  else if (mode == Mode::effort)
  {
    setEffort(command.effort);
  }
  else
  {
    throw std::runtime_error("[setPoint]: Invalid mode selected");
  }
}

void VREPDriverBase::commandCallback(const vrep_msgs::VREPDriverMessageConstPtr &message)
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

void VREPDriverBase::initialize()
{
  joint_handle = sim->getObjectHandle(joint_name);
  link_handle = sim->getObjectHandle(link_name);
  initializeChild();
  sim->info("[initialize]: Found an id of " + to_string(joint_handle) + " for \"" + joint_name + "\"");
  sim->info("[initialize]: Found an id of " + to_string(link_handle) + " for \"" + link_name + "\"");
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


