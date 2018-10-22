#include <vrep_driver_access/vrep_driver_access.h>
#include <driver_access/mode.h>
#include <string>

using namespace driver_access;

using vrep_msgs::VREPDriverMessage;
using vrep_msgs::VREPDriverMessageConstPtr;
using std::to_string;
using vrep_msgs::PIDGet;
using vrep_msgs::PIDSet;

VREPDriverAccess::VREPDriverAccess(const Limits &limits, ID id, Mode mode) : DriverAccess(limits, id, mode)
{
  queue.reset(new ros::CallbackQueue);
  nh.reset(new ros::NodeHandle("/vrep/" + name(id)));
  nh->setCallbackQueue(queue.get());

  subscriber.reset(new ros::Subscriber);
  publisher.reset(new ros::Publisher);
  pid_get_client.reset(new ros::ServiceClient);
  pid_set_client.reset(new ros::ServiceClient);

  (*subscriber) = nh->subscribe("state", 1, &VREPDriverAccess::callback, this);
  (*publisher) = nh->advertise<VREPDriverMessage>("command", 1, true);
  (*pid_set_client) = nh->serviceClient<PIDGet>("get_pid");
  (*pid_get_client) = nh->serviceClient<PIDSet>("set_pid");

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

void VREPDriverAccess::callback(const vrep_msgs::VREPDriverMessageConstPtr &message)
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
  VREPDriverMessage command;
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
  VREPDriverMessage command;
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
  VREPDriverMessage command;
  command.header = getHeader();
  command.id = static_cast<uint8_t>(id);
  command.mode = static_cast<uint8_t>(Mode::effort);
  command.velocity = 0;
  command.effort = effort;
  command.position = 0;
  publisher->publish(command);
}

void VREPDriverAccess::setPID(double p, double i, double d)
{
  PIDSet msg;
  msg.request.p = p;
  msg.request.i = i;
  msg.request.d = d;
  pid_set_client->call(msg);
  if (!msg.response.success)
  {
    ROS_WARN("Unable to set PID");
  }
}

tuple3d VREPDriverAccess::getPID()
{
  PIDGet msg;
  pid_get_client->call(msg);
  if (msg.response.success)
  {
    return std::make_tuple(msg.response.p, msg.response.i, msg.response.d);
  }
  else
  {
    ROS_WARN("Unable to get PID");
  }
  return std::make_tuple(-1.0, -1.0, -1.0);
}

