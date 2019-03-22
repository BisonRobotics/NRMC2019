#ifndef STEPPER_STEPPER_H
#define STEPPER_STEPPER_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <string>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <stepper/stepper_types.h>

namespace stepper
{
  typedef struct can_frame can_frame_t;

/** CAN ID Format
 * [0:5] ID (64 IDs)
 * [6:10] Message Type (32 Message Types)
 */

/** Message types
 * - Set zero
 * - Set position
 * - Scan
 * - Set velocity
 * - Send Error
 */
  struct State
  {
    State(double position, double velocity);
    const double position;
    const double velocity;
  };

  class Stepper
  {
  public:
    Stepper(std::string network, canid_t tx_id, canid_t rx_id);
    bool messagesAvailable();
    can_frame_t receiveFrame();
    void sendFrame(can_frame_t frame);
    canid_t getCanID(uint32_t id, uint32_t messageType);
    MessageType getMessageType(canid_t id);
    void scan();
    void requestState();
    State poll();
    void findZero();
    void setZero();
    void setMode(Mode mode);
    void setLimits(double ccw, double cw);
    void setPoint(double value);



  private:
    can_frame_t rx_frame;
    can_frame_t tx_frame;
    int can_socket;
    canid_t tx_id, rx_id;
    struct sockaddr_can address;
    struct ifreq request;
    struct can_filter filters[1];
  };
}


#endif //STEPPER_STEPPER_H
