#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>

#include <stepper/stepper.h>
#include <stdexcept>

using namespace stepper;

Stepper::Stepper(std::string network, uint tx_id, uint rx_id) :
  tx_id(tx_id), rx_id(rx_id), seq(0)
{
  request = new ifreq;
  address = new sockaddr_can;
  filters = new can_filter[1];

  can_socket = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  strcpy(request->ifr_name, network.c_str());
  ioctl(can_socket, SIOCGIFINDEX, request);
  address->can_family = AF_CAN;
  address->can_ifindex = request->ifr_ifindex;

  if (bind(can_socket, (struct sockaddr*)address, sizeof(*address)) < 0)
  {
    throw std::runtime_error("Unable to bind to socket");
  }

  filters[0].can_id = rx_id;
  filters[0].can_mask = 0xF; // ID is contained in first 4 bits

  if (setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(*filters)) < 0)
  {
    throw std::runtime_error("Unable to set socket options");
  }
}

Stepper::~Stepper()
{
  delete request;
  delete address;
  delete[] filters;
}

bool Stepper::messagesAvailable()
{
  char tmp;
  return recv(can_socket, &tmp, sizeof(char), MSG_PEEK) > 0;
}

can_frame Stepper::receiveFrame()
{
  // Read message
  can_frame frame;
  ssize_t bytes = read(can_socket, &frame, sizeof(struct can_frame));

  // Verify
  if (bytes < 0)
  {
    throw std::runtime_error("Socket read failed");
  }
  if (bytes < sizeof(struct can_frame))
  {
    throw std::runtime_error("Incomplete socket read");
  }

  return frame;
}

void Stepper::sendFrame(can_frame frame)
{
  size_t frame_size = sizeof(struct can_frame);
  ssize_t bytes = write(can_socket, &frame, frame_size);
  if (bytes < frame_size)
  {
    throw std::runtime_error("Incomplete socket write");
  }
}

canid_t Stepper::generateCanID(uint32_t id, uint32_t messageType)
{
  return (canid_t) id | messageType << 4;
}

void Stepper::requestState()
{
  seq++;
  can_frame frame;
  frame.can_id = generateCanID(tx_id, MessageType::RequestState);
  frame.can_dlc = 0;
  sendFrame(frame);
}

State Stepper::pollState()
{
  requestState();
  for(int tries = 0; tries < 10; tries++) // Try for one second and then quite
  {
    if (messagesAvailable())
    {
      can_frame frame = receiveFrame();
      if (getMessageType(frame.can_id) == MessageType::StateMessage)
      {
        // TODO do conversion
        float position, velocity;
        memcpy(&position, &frame.data[0], 4);
        memcpy(&velocity, &frame.data[4], 4);
        return State{position, velocity};
      }
      else
      {
        printf("[WARN] invalid message");
      }
    }
    usleep(1000);
  }
  throw std::runtime_error("Unable to poll stepper state");
}

void Stepper::findZero()
{
  can_frame frame;
  frame.can_id = generateCanID(tx_id, MessageType::FindZero);
  frame.can_dlc = 0;
  sendFrame(frame);
}

void Stepper::setZero()
{
  can_frame frame;
  frame.can_id = generateCanID(tx_id, MessageType::SetZero);
  frame.can_dlc = 0;
  sendFrame(frame);
}

void Stepper::setMode(Mode mode, float set_point)
{
  can_frame frame;
  frame.can_id = generateCanID(tx_id, MessageType::SetMode);
  frame.can_dlc = 5;
  frame.data[0] = mode;
  memcpy(&frame.data[1], &set_point, 4);
  sendFrame(frame);
}

void Stepper::setLimits(float ccw, float cw)
{
  can_frame frame;
  frame.can_id = generateCanID(tx_id, MessageType::SetLimits);
  frame.can_dlc = 8;
  memcpy(&frame.data[0], &ccw, 4);
  memcpy(&frame.data[4], &cw, 4);
  sendFrame(frame);
}

void Stepper::setPoint(float value)
{
  can_frame frame;
  frame.can_id = generateCanID(tx_id, MessageType::SetPoint);
  frame.can_dlc = 4;
  memcpy (&frame.data[0], &value, 4);
  sendFrame(frame);
}

MessageType Stepper::getMessageType(canid_t id)
{
  return (MessageType)(id >> 4);
}

uint Stepper::getSeq()
{
  return seq;
}

State::State(double position, double velocity) : position(position), velocity(velocity){}

State::State() : position(0.0), velocity(0.0) {};
