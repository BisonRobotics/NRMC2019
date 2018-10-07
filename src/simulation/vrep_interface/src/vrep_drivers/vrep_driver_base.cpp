#include <vrep_drivers/vrep_driver_base.h>
#include <driver_access/mode.h>
#include <driver_access/limits.h>

using namespace vrep_interface;

using std::to_string;
using vrep_msgs::VREPDriverMessage;
using vrep_msgs::VREPDriverMessageConstPtr;
using driver_access::Limits;
using driver_access::Mode;

VREPDriverBase::VREPDriverBase(uint8_t id, const std::string &joint_name) :
    DriverAccess(Limits(-1e10, 1e10, 0, 1e10, 0, 1e10), id),
    joint_name(joint_name)
{
  queue.reset(new ros::CallbackQueue);
  nh.reset(new ros::NodeHandle("/vrep/driver" + to_string(id)));
  nh->setCallbackQueue(queue.get());

  subscriber.reset(new ros::Subscriber);
  publisher.reset(new ros::Publisher);

  (*subscriber) = nh->subscribe("command", 10, &VREPDriverBase::callback, this);
  (*publisher) = nh->advertise<VREPDriverMessage>("state", 10, true);

  spinner.reset(new ros::AsyncSpinner(0, queue.get()));
  spinner->start();

  state.id = id;
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

driver_access::Mode VREPDriverBase::getMode()
{
  return static_cast<driver_access::Mode>(command.mode);
}

void VREPDriverBase::callback(const vrep_msgs::VREPDriverMessageConstPtr &message)
{
  if (message->header.stamp > command.header.stamp)
  {
    command = *message;

    if (getMode() == driver_access::Mode::position)
    {
      setPosition(command.position);
    }
    else if (getMode() == driver_access::Mode::velocity)
    {
      setVelocity(command.velocity);
    }
    else if (getMode() == driver_access::Mode::effort)
    {
      setEffort(command.effort);
    }
    else
    {
      throw std::runtime_error("Invalid mode");
    }
  }
}

void VREPDriverBase::updateHeader(std_msgs::Header *header)
{
  header->stamp = ros::Time::now();
  header->seq = seq++;
}

void VREPDriverBase::updateHandle()
{
  handle = simGetObjectHandle(joint_name.c_str());
  if (handle == -1)
  {
    throw std::runtime_error("[VREPWheelDriver Constructor] Unable to find joint");
  }
  simAddStatusbarMessage(("[method updateWheelIDs] Found an id of " + std::to_string(handle)
                          + " for \"" + std::string(joint_name) + "\"").c_str());
}

void VREPDriverBase::shutdown()
{
  spinner->stop();
  subscriber->shutdown();
  publisher->shutdown();
  nh->shutdown();
  queue->clear();
}


