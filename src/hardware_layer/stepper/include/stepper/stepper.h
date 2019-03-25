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
 * [0:3] ID (16 IDs)
 * [4:10] Message Type (128 Message Types)
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
    void requestState();
    State pollState();
    void findZero();
    void setZero();
    void setMode(Mode mode, float initial_setpoint);
    void setLimits(double ccw, double cw);
    void setPoint(double value);

    static canid_t generateCanID(uint32_t id, uint32_t messageType);
    static MessageType getMessageType(canid_t id);




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
