#include <stepper/stepper.h>
#include <stdexcept>

using namespace stepper;

Stepper::Stepper(std::string network, canid_t tx_id, canid_t rx_id) :
  tx_id(tx_id), rx_id(rx_id)
{
  can_socket = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  strcpy(request.ifr_name, network.c_str());
  ioctl(can_socket, SIOCGIFINDEX, &request);
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;

  if (bind(can_socket, (struct sockaddr *)&address, sizeof(address)) < 0)
  {
    throw std::runtime_error("Unable to bind to socket");
  }

  filters[0].can_id = rx_id;
  filters[0].can_mask = CAN_SFF_MASK;

  if (setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters)) < 0)
  {
    throw std::runtime_error("Unable to set socket options");
  }
}

bool Stepper::messagesAvailable()
{
  char tmp;
  return recv(can_socket, &tmp, sizeof(char), MSG_PEEK) > 0;
}

can_frame_t Stepper::receiveFrame()
{
  // Read message
  ssize_t bytes = read(can_socket, &rx_frame, sizeof(struct can_frame));

  // Verify
  if (bytes < 0)
  {
    throw std::runtime_error("Socket read failed");
  }
  if (bytes < sizeof(struct can_frame))
  {
    throw std::runtime_error("Incomplete socket read");
  }

  return rx_frame;
}

void Stepper::sendFrame(can_frame_t frame)
{
  size_t frame_size = sizeof(struct can_frame);
  ssize_t bytes = write(can_socket, &frame, frame_size);
  if (bytes < frame_size)
  {
    throw std::runtime_error("Incomplete socket write");
  }
}

canid_t Stepper::getCanID(uint32_t id, uint32_t messageType)
{
  return (canid_t) id | messageType << 6;
}

void Stepper::scan()
{
  tx_frame.can_id = getCanID(tx_id, MessageType::Scan);
  tx_frame.can_dlc = 0;
  sendFrame(tx_frame);
}

void Stepper::requestState()
{
  tx_frame.can_id = getCanID(tx_id, MessageType::RequestState);
  tx_frame.can_dlc = 0;
  sendFrame(tx_frame);
}

State Stepper::poll()
{
  requestState();
  while(true)
  {
    if (messagesAvailable())
    {
      can_frame_t frame = receiveFrame();
      if (getMessageType(frame.can_id) == MessageType::StateMessage)
      {
        // TODO do conversion
        return State{-1.0, -1.0};
      }
    }
    usleep(1000);
  }

}

void Stepper::findZero()
{
  tx_frame.can_id = getCanID(tx_id, MessageType::FindZero);
  tx_frame.can_dlc = 0;
  sendFrame(tx_frame);
}

void Stepper::setZero()
{
  tx_frame.can_id = getCanID(tx_id, MessageType::SetZero);
  tx_frame.can_dlc = 0;
  sendFrame(tx_frame);
}

void Stepper::setMode(Mode mode)
{
  tx_frame.can_id = getCanID(tx_id, MessageType::SetMode);
  tx_frame.can_dlc = 1;
  tx_frame.data[0] = mode;
  sendFrame(tx_frame);
}

void Stepper::setLimits(double ccw, double cw)
{
  // TODO do some double to 16 bit conversions
  tx_frame.can_id = getCanID(tx_id, MessageType::SetLimits);
  tx_frame.can_dlc = 4;
  tx_frame.data[0] = 0;
  tx_frame.data[1] = 0;
  tx_frame.data[2] = 0;
  tx_frame.data[3] = 0;
  sendFrame(tx_frame);
}

void Stepper::setPoint(double value)
{
  // TODO do some double to 16 bit conversions
  tx_frame.can_id = getCanID(tx_id, MessageType::SetPoint);
  tx_frame.can_dlc = 2;
  tx_frame.data[0] = 0;
  tx_frame.data[1] = 0;
  sendFrame(tx_frame);
}

MessageType Stepper::getMessageType(canid_t id)
{
  // TODO actually do this
  return Scan;
}

State::State(double position, double velocity) : position(position), velocity(velocity){};