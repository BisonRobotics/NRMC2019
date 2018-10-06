#include <vrep_driver_access/vrep_driver_access.h>
#include <driver_access/mode.h>

using namespace driver_access;

using std::to_string;
using vrep_msgs::VREPDriverMessage;
using vrep_msgs::VREPDriverMessageConstPtr;

VREPDriverAccess::VREPDriverAccess(const Limits &limits, uint8_t id) :
    DriverAccessBase(limits, id, "vrep_driver_" + to_string(id))
{
  queue.reset(new ros::CallbackQueue);
  nh.reset(new ros::NodeHandle("/vrep/driver" + to_string(id)));
  nh->setCallbackQueue(queue.get());

  subscriber.reset(new ros::Subscriber);
  publisher.reset(new ros::Publisher);

  (*subscriber) = nh->subscribe("status", 10, &VREPDriverAccess::callback, this);
  (*publisher) = nh->advertise<VREPDriverMessage>("command", 10, true);

  spinner.reset(new ros::AsyncSpinner(0, queue.get()));
  spinner->start();
}

double VREPDriverAccess::getVelocity()
{
  return current.velocity;
}

double VREPDriverAccess::getTorque()
{
  return current.torque;
}

double VREPDriverAccess::getPosition()
{
  return current.position;
}

void VREPDriverAccess::callback(const vrep_msgs::VREPDriverMessageConstPtr &message)
{
  if (message->header.stamp > current.header.stamp)
  {
    current = *message;
  }
}

std_msgs::Header VREPDriverAccess::getHeader()
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.seq = seq++;
  header.frame_id = name;
}

void VREPDriverAccess::setDriverVelocity(double velocity)
{
  VREPDriverMessage command;
  command.header = getHeader();
  command.id = id;
  command.mode = Mode::velocity;
  command.velocity = velocity;
  command.torque = 0;
  command.position = 0;
  publisher->publish(command);
}

void VREPDriverAccess::setDriverTorque(double torque)
{
  VREPDriverMessage command;
  command.header = getHeader();
  command.id = id;
  command.mode = Mode::velocity;
  command.velocity = 0;
  command.torque = torque;
  command.position = 0;
  publisher->publish(command);
}

void VREPDriverAccess::setDriverPosition(double position)
{
  VREPDriverMessage command;
  command.header = getHeader();
  command.id = id;
  command.mode = Mode::velocity;
  command.velocity = 0;
  command.torque = 0;
  command.position = position;
  publisher->publish(command);
};

