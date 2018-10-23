#include <vrep_driver_access/vrep_driver_access.h>
#include <driver_access/mode.h>
#include <string>

using namespace driver_access;

using vrep_msgs::DriverMessage;
using vrep_msgs::DriverMessageConstPtr;
using std::to_string;

VREPDriverAccess::VREPDriverAccess(const Limits &limits, ID id, Mode mode) : DriverAccess(limits, id, mode)
{
  queue.reset(new ros::CallbackQueue);
  nh.reset(new ros::NodeHandle("/vrep/" + name(id)));
  nh->setCallbackQueue(queue.get());

  subscriber.reset(new ros::Subscriber);
  publisher.reset(new ros::Publisher);

  (*subscriber) = nh->subscribe("state", 1, &VREPDriverAccess::callback, this);
  (*publisher) = nh->advertise<DriverMessage>("command", 1, true);

  spinner.reset(new ros::AsyncSpinner(0, queue.get()));
  spinner->start();
}

double VREPDriverAccess::getPosition()
{
  return state.position;
}

double VREPDriverAccess::getVelocity()
{
  return state.velocity;
}

double VREPDriverAccess::getEffort()
{
  return state.effort;
}

void VREPDriverAccess::callback(const vrep_msgs::DriverMessageConstPtr &message)
{
  if (message->header.stamp > state.header.stamp)
  {
    state = *message;
  }
}

std_msgs::Header VREPDriverAccess::getHeader()
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.seq = seq++;
  return header;
}

void VREPDriverAccess::setDriverPosition(double position)
{
  DriverMessage command;
  command.header = getHeader();
  command.id = static_cast<uint8_t>(id);
  command.mode = static_cast<uint8_t>(Mode::position);
  command.velocity = 0;
  command.effort = 0;
  command.position = position;
  publisher->publish(command);
}

void VREPDriverAccess::setDriverVelocity(double velocity)
{
  DriverMessage command;
  command.header = getHeader();
  command.id = static_cast<uint8_t>(id);
  command.mode = static_cast<uint8_t>(Mode::velocity);
  command.velocity = velocity;
  command.effort = 0;
  command.position = 0;
  publisher->publish(command);
}

void VREPDriverAccess::setDriverEffort(double effort)
{
  DriverMessage command;
  command.header = getHeader();
  command.id = static_cast<uint8_t>(id);
  command.mode = static_cast<uint8_t>(Mode::effort);
  command.velocity = 0;
  command.effort = effort;
  command.position = 0;
  publisher->publish(command);
}

