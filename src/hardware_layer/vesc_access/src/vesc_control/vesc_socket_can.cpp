#include "vesc_control/vesc_socket_can.h"
#include <byteswap.h>
#include <cmath>

Vesc::Vesc(char *interface, uint8_t controllerID, std::string name) : Vesc(interface, controllerID, 0, name)
{
  // init_socketCAN(interface);
  // _controllerID = controllerID;
}

Vesc::Vesc(char *interface, uint8_t controllerID, uint32_t quirks, std::string name) :
  rpm_(0), current_(0.0f), duty_cycle_(0.0f), position_(0.0f), tachometer_(0), watt_hours_(0.0f),
  in_current_(0.0f), vin_(0.0f), motor_temp_(0.0f), pcb_temp_(0.0f), encoder_index_(false),
  adc_(0), flimit_(false), rlimit_(false), fault_code_(mc_fault_code::FAULT_CODE_NONE),
  state_(mc_state::MC_STATE_OFF)
{
  this->name = name;
  this->flimit_=false;
  this->rlimit_=false;
  //first_time = true;
  init_socketCAN(interface);
  controller_id_ = controllerID;
  quirks_ = quirks;
  gettimeofday(&previous_message_time_, NULL);  // initialize _prevmsgtime with something
  previous_message_time_.tv_sec -= 1;           // make it in the past to avoid false positives
}

void Vesc::init_socketCAN(char *ifname)
{
  s_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);  // create nonblocking raw can socket
  if (s_ == -1)
  {
    throw VescException(this->name + " Unable to create raw CAN socket");
  }
  strcpy(ifr_.ifr_name, ifname);
  if (ioctl(s_, SIOCGIFINDEX, &ifr_))
  {
    throw VescException(this->name + " Error creating interface");
  }
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;

  int ret = bind(s_, (struct sockaddr *)&addr_, sizeof(addr_));
  if (ret == -1)
  {
    throw VescException(this->name + " Unable to bind raw CAN socket");
  }

  sbcm_ = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
  if (sbcm_ == -1)
  {
    throw VescException(this->name + " Unable to create bcm socket");
  }
  ret = connect(sbcm_, (struct sockaddr *)&addr_, sizeof(addr_));

  if (ret == -1)
  {
    throw VescException(this->name + " Unable to connect bcm socket");
  }
}

// figure out whether or not a destructor is needed
// destructor is not needed because c++ objects should handle it

struct can_msg
{
  struct bcm_msg_head msg_head;
  struct can_frame frame[1];
} msg;
void Vesc::setPoint(mc_control_mode mode, float setpoint, uint index)
{
  if (enable_)
  {
    custom_control set;
    set.setpointf = setpoint;
    if (mode == mc_control_mode::CONTROL_MODE_CUSTOM)
    {
      set.control_mode = mode + index;
    }
    else
    {
      set.control_mode = mode;
    }
    // struct can_frame frame;
    // frame.can_id = mode << 8 | _controllerID | 0x80000000;
    // frame.can_dlc = sizeof(VESC_set);
    // memcpy(frame.data, &set, sizeof(VESC_set));

    // write(s, &frame, sizeof(struct can_frame));

    msg.msg_head.opcode = TX_SETUP;
    msg.msg_head.can_id = CAN_PACKET_CONTROL << 8 | controller_id_ | 0x80000000;
    msg.msg_head.flags = SETTIMER | STARTTIMER | TX_CP_CAN_ID;
    msg.msg_head.nframes = 1;
    msg.msg_head.count = 0;
    msg.msg_head.ival1.tv_sec = 0;
    msg.msg_head.ival1.tv_usec = 0;
    msg.msg_head.ival2.tv_sec = 0;
    msg.msg_head.ival2.tv_usec = 1000 * 10;
    // msg.frame[0].can_id    = 0x42; /* obsolete when using TX_CP_CAN_ID */
    msg.frame[0].can_dlc = sizeof(custom_control);
    memcpy(msg.frame[0].data, &set, 8);
    write(sbcm_, &msg, sizeof(msg));
  }
}

void Vesc::setDuty(float dutyCycle)
{
  setPoint(CONTROL_MODE_DUTY, dutyCycle);
}
void Vesc::setCurrent(float current)
{
  setPoint(CONTROL_MODE_CURRENT, current);
}
void Vesc::setCurrentBrake(float current)
{
  setPoint(CONTROL_MODE_CURRENT_BRAKE, current);
}
void Vesc::setRpm(float rpm)
{
  setPoint(CONTROL_MODE_SPEED, rpm);
}
void Vesc::setPos(float pos)
{
  setPoint(CONTROL_MODE_POS, pos);
}
void Vesc::setCustom(float setpoint, uint index)
{
  setPoint(CONTROL_MODE_CUSTOM, setpoint, index);
}

void Vesc::setCustom(float setpoint)
{
  setPoint(CONTROL_MODE_CUSTOM, setpoint, 0);
}

void Vesc::enable()
{
  enable_ = 1;
}
void Vesc::disable()
{
  setCurrent(0);
  enable_ = 0;
}

void Vesc::processMessages()
{
  struct can_frame msg;
  while (ros::ok())
  {
    int a = read(s_, &msg, sizeof(msg));
    if (a == -1)
      break;
    if ((msg.can_id & ~0x80000000 & 0xFF) == controller_id_)
    {
      // std::cout << "canid " << std::hex << (msg.can_id & ~0x80000000 & 0xFF) << std::dec << std::endl;
      // std::cout << "canid " << std::hex << (msg.can_id) << std::dec << std::endl;

      switch ((msg.can_id & ~0x80000000 & ~controller_id_) >> 8)
      {
        case CAN_PACKET_STATUS:  // default status message, probably going to be unused but we can handle it if it does
                                 // appear
          // received data is big endian
          //_rpm = __bswap_32((*(VESC_status*) msg.data).rpm); // pointer casting!
          //_current = ((int16_t) __bswap_16((*(VESC_status*) msg.data).current)) / 10.0;
          duty_cycle_ = ((int16_t)__bswap_16((*(VESC_status *)msg.data).duty_cycle)) / 1000.0f;
          gettimeofday(&previous_message_time_, NULL);
          break;
        case CAN_PACKET_STATUS1:  // custom status message

            rpm_ = (*(VESC_status1 *)msg.data).rpm;


          current_ = (*(VESC_status1 *)msg.data).motorCurrent / 10.0f;
          position_ = (*(VESC_status1 *)msg.data).position / 1000.0f;
          gettimeofday(&previous_message_time_, NULL);
          break;
        case CAN_PACKET_STATUS2:
          tachometer_ = (*(VESC_status2 *)msg.data).tachometer;
          adc_ = (*(VESC_status2 *)msg.data).adc;
          flimit_ = (bool)(*(VESC_status2 *)msg.data).flimit;
          rlimit_ = (bool)(*(VESC_status2 *)msg.data).rlimit;
          gettimeofday(&previous_message_time_, NULL);
          break;
        case CAN_PACKET_STATUS3:
          watt_hours_ = (*(VESC_status3 *)msg.data).wattHours;
          in_current_ = (*(VESC_status3 *)msg.data).inCurrent / 100.0f;
          vin_ = (*(VESC_status3 *)msg.data).voltage;
          gettimeofday(&previous_message_time_, NULL);
          break;
        case CAN_PACKET_STATUS4:
          motor_temp_ = (*(VESC_status4 *)msg.data).tempMotor;
          pcb_temp_ = (*(VESC_status4 *)msg.data).tempPCB;
          fault_code_ = (mc_fault_code)(*(VESC_status4 *)msg.data).faultCode;
          state_ = (mc_state)(*(VESC_status4 *)msg.data).state;
          encoder_index_ = (bool)(*(VESC_status4 *)msg.data).encoderIndex;
          gettimeofday(&previous_message_time_, NULL);
          break;
        default:
          break;
      }
    }
  }
}

int Vesc::getRpm()
{
  processMessages();
  return rpm_;
}

float Vesc::getCurrent()
{
  processMessages();
  return current_;
}

float Vesc::getDutyCycle()
{
  processMessages();
  return duty_cycle_;
}
float Vesc::getPosition()
{
  processMessages();
  return position_;
}
int Vesc::getTachometer()
{
  processMessages();
  return tachometer_;
}
float Vesc::getWattHours()
{
  processMessages();
  return watt_hours_;
}
float Vesc::getInCurrent()
{
  processMessages();
  return in_current_;
}
#define VIN_R1 39000.0
#define VIN_R2 2200.0
#define V_REG 3.3
#define GET_INPUT_VOLTAGE(adc_val) ((V_REG / 4095.0) * (float)adc_val * ((VIN_R1 + VIN_R2) / VIN_R2))
float Vesc::getVin()
{
  processMessages();
  return GET_INPUT_VOLTAGE(vin_);
}
#define NTC_RES_GND(adc_val) (10000.0 * adc_val / 4095.0) / (1 - adc_val / 4095.0)
#define NTC_RES(adc_val) ((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_val)                                                                                              \
  (1.0 / ((log(NTC_RES(adc_val) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15)  // use when ntc is connected to vcc
#define NTC_TEMP_1k_THERM(adc_val)                                                                                     \
  (1.0 / ((log(NTC_RES(adc_val) / 1000.0) / 3434.0) + (1.0 / 298.15)) - 273.15)  // use when ntc is connected to vcc
#define NTC_TEMP_GND(adc_val)                                                                                          \
  (1.0 / ((log(NTC_RES_GND(adc_val) / 10000.0) / 3434.0) + (1.0 / 298.15)) -                                           \
   273.15)  // use when ntc is connected to ground
float Vesc::getTempMotor()
{
  processMessages();
  return NTC_TEMP_GND(motor_temp_);
}
float Vesc::getTempPCB()
{
  processMessages();
  if (quirks_ == 1)
    return NTC_TEMP_1k_THERM(pcb_temp_);
  return NTC_TEMP(pcb_temp_);
}
Vesc::mc_fault_code Vesc::getFaultCode()
{
  processMessages();
  return fault_code_;
}
Vesc::mc_state Vesc::getState()
{
  processMessages();
  return state_;
}

void Vesc::resetWattHours()
{
  custom_config_data config;
  config.config_enum = RESET_WATT_HOURS;

  struct can_frame frame;
  frame.can_id = CAN_PACKET_CONFIG << 8 | controller_id_ | 0x80000000;
  frame.can_dlc = sizeof(custom_config_data);
  memcpy(frame.data, &config, sizeof(custom_config_data));

  write(s_, &frame, sizeof(struct can_frame));
}
bool Vesc::encoderIndexFound()
{
  processMessages();
  return encoder_index_;
}
int timediffms(struct timeval tv, struct timeval last_tv)
{
  // stolen from candump.c
  struct timeval diff;
  diff.tv_sec = tv.tv_sec - last_tv.tv_sec;
  diff.tv_usec = tv.tv_usec - last_tv.tv_usec;
  if (diff.tv_usec < 0)
    diff.tv_sec--, diff.tv_usec += 1000000;
  if (diff.tv_sec < 0)
    diff.tv_sec = diff.tv_usec = 0;
  return diff.tv_sec * 1000 + diff.tv_usec / 1000;
}

bool Vesc::isAlive()
{
  processMessages();
  struct timeval now;
  gettimeofday(&now, NULL);
  return timediffms(now, previous_message_time_) < 100;  // must have received a message in the last 100 ms
}

bool Vesc::getForLimit()
{
  processMessages();
  return flimit_;
}

bool Vesc::getRevLimit()
{
  processMessages();
  return rlimit_;
}

int Vesc::getADC()
{
  processMessages();
  return adc_;
}
