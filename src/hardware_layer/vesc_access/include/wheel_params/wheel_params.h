#ifndef WHEEL_PARAMS_H
#define WHEEL_PARAMS_H

#include "vesc_access/ivesc_access.h"

#define WHEEL_CAN_NETWORK ("vesc_can")

#define FRONT_LEFT_WHEEL_ID 1
#define FRONT_RIGHT_WHEEL_ID 2
#define BACK_RIGHT_WHEEL_ID 3
#define BACK_LEFT_WHEEL_ID 4

#define FRONT_LEFT_WHEEL_DIRECTION -1.0f
#define FRONT_RIGHT_WHEEL_DIRECTION 1.0f
#define BACK_RIGHT_WHEEL_DIRECTION -1.0f
#define BACK_LEFT_WHEEL_DIRECTION 1.0f

#define MAX_WHEEL_VELOCITY .5f
#define MAX_WHEEL_TORQUE 176.0f
#define MAX_WHEEL_DUTY .4f
#define WHEEL_GEAR_RATIO 179.83f
#define WHEEL_OUTPUT_RATIO 15.95929e-3f // (6 in)*(0.0254 m/in)*(2*pi rad/rot)/(60 s/min)
#define WHEEL_POLE_PAIRS 10
#define WHEEL_TORQUE_CONSTANT .02120f

#define ROBOT_AXLE_LENGTH 0.64f
#define ROBOT_MAX_SPEED 0.5f

nsVescAccess::vesc_param_struct_t front_left_param =
{
  .direction = FRONT_LEFT_WHEEL_DIRECTION,
  .max_velocity = MAX_WHEEL_VELOCITY,
  .max_torque = MAX_WHEEL_TORQUE,
  .max_duty = MAX_WHEEL_DUTY,
  .gear_ratio = WHEEL_GEAR_RATIO,
  .output_ratio = FRONT_LEFT_WHEEL_DIRECTION * WHEEL_OUTPUT_RATIO,
  .pole_pairs = WHEEL_POLE_PAIRS,
  .torque_constant = WHEEL_TORQUE_CONSTANT,
  WHEEL_CAN_NETWORK,
  .can_id = FRONT_LEFT_WHEEL_ID,
  .name = "front_left_wheel"
};

nsVescAccess::vesc_param_struct_t front_right_param =
{
  .direction = FRONT_RIGHT_WHEEL_DIRECTION,
  .max_velocity = MAX_WHEEL_VELOCITY,
  .max_torque = MAX_WHEEL_TORQUE,
  .max_duty = MAX_WHEEL_DUTY,
  .gear_ratio = WHEEL_GEAR_RATIO,
  .output_ratio = FRONT_RIGHT_WHEEL_DIRECTION * WHEEL_OUTPUT_RATIO,
  .pole_pairs = WHEEL_POLE_PAIRS,
  .torque_constant = WHEEL_TORQUE_CONSTANT,
  WHEEL_CAN_NETWORK,
  .can_id = FRONT_RIGHT_WHEEL_ID,
  .name = "front_right_wheel"
};

nsVescAccess::vesc_param_struct_t back_right_param =
{
  .direction = BACK_RIGHT_WHEEL_DIRECTION,
  .max_velocity = MAX_WHEEL_VELOCITY,
  .max_torque = MAX_WHEEL_TORQUE,
  .max_duty = MAX_WHEEL_DUTY,
  .gear_ratio = WHEEL_GEAR_RATIO,
  .output_ratio = BACK_RIGHT_WHEEL_DIRECTION * WHEEL_OUTPUT_RATIO,
  .pole_pairs = WHEEL_POLE_PAIRS,
  .torque_constant = WHEEL_TORQUE_CONSTANT,
  WHEEL_CAN_NETWORK,
  .can_id = BACK_RIGHT_WHEEL_ID,
  .name = "back_right_wheel"
};

nsVescAccess::vesc_param_struct_t back_left_param =
{
  .direction = BACK_LEFT_WHEEL_DIRECTION,
  .max_velocity = MAX_WHEEL_VELOCITY,
  .max_torque = MAX_WHEEL_TORQUE,
  .max_duty = MAX_WHEEL_DUTY,
  .gear_ratio = WHEEL_GEAR_RATIO,
  .output_ratio =  BACK_LEFT_WHEEL_DIRECTION * WHEEL_OUTPUT_RATIO,
  .pole_pairs = WHEEL_POLE_PAIRS,
  .torque_constant = WHEEL_TORQUE_CONSTANT,
  WHEEL_CAN_NETWORK,
  .can_id = BACK_LEFT_WHEEL_ID,
  .name = "back_left_wheel"
};

#endif