#ifndef STEPPER_STEPPER_H
#define STEPPER_STEPPER_H

#include <string>

#include <stepper/stepper_types.h>

namespace stepper
{

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
    State();
    State(double position, double velocity);
    double position;
    double velocity;
  };

  class Stepper
  {
  public:
    Stepper(std::string network, uint tx_id, uint rx_id);
    ~Stepper();
    bool messagesAvailable();
    struct can_frame receiveFrame();
    void sendFrame(struct can_frame frame);
    void requestState();
    State pollState();
    void findZero();
    void setZero();
    void setMode(Mode mode, float initial_setpoint);
    void setLimits(float ccw, float cw);
    void setPoint(float value);
    uint getSeq();

    static uint generateCanID(uint32_t id, uint32_t messageType);
    static MessageType getMessageType(uint id);

  private:
    int can_socket;
    uint tx_id, rx_id, seq;
    struct sockaddr_can *address;
    struct ifreq *request;
    struct can_filter *filters;
  };
}


#endif //STEPPER_STEPPER_H
