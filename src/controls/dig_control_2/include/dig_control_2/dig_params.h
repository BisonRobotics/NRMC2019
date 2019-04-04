#ifndef DIG_CONTROL_2_DIG_PARAMS_H
#define DIG_CONTROL_2_DIG_PARAMS_H

#include "vesc_access/ivesc_access.h"

#define VESC_CAN_NETWORK ("vesc_can")

#define CENTRAL_DRIVE_ID 5
#define BACKHOE_ACTUATOR_ID 6
#define VIBRATOR_ID 7
#define BUCKET_ACTUATOR_ID 8

#define BACKHOE_ACTUATOR_DIRECTION 1.0f
#define CENTRAL_DRIVE_DIRECTION 1.0f

#define MAX_CENTRAL_DRIVE_VELOCITY 0.9425f  // in rad/s
#define MAX_CENTRAL_DRIVE_TORQUE 10.0f     // in Nm -> the real number is 675 but we should never approach that
#define MAX_CENTRAL_DRIVE_DUTY .4f
#define CENTRAL_DRIVE_GEAR_RATIO -1.0f
#define CENTRAL_DRIVE_OUTPUT_RATIO 1.0f  //  purely rotational
#define CENTRAL_DRIVE_POLE_PAIRS 5
#define CENTRAL_DRIVE_TORQUE_CONSTANT 1.0f  // Nm/A

#define MAX_BACKHOE_ACTUATOR_VELOCITY .05f  // this should be change
#define MAX_BACKHOE_ACTUATOR_TORQUE 4.0f
#define MAX_LINEAR_DUTY .4
#define BACKHOE_ACTUATOR_GEAR_RATIO 1
#define BACKHOE_ACTUATOR_OUTPUT_RATIO ((float)(1.0/(5.206E6)))  // this should be the pitch of the screw in m/s
#define BACKHOE_ACTUATOR_POLE_PAIRS 1
#define BACKHOE_ACTUATOR_TORQUE_CONSTANT 1.0f

#define MAX_VIBRATOR_VELOCITY 6000.0f   // in erpm
#define MAX_VIBRATOR_TORQUE 11.0f    // in Amps
#define MAX_VIBRATOR_DUTY .5f
#define VIBRATOR_GEAR_RATIO 1.0f
#define VIBRATOR_OUTPUT_RATIO 1.0f
#define VIBRATOR_POLE_PAIRS 1
#define VIBRATOR_TORQUE_CONSTANT 1.0f

#define MAX_BUCKET_ACTUATOR_VELOCITY 10.0f
#define MAX_BUCKET_ACTUATOR_TORQUE 10.0f
#define MAX_BUCKET_ACTUATOR_DUTY .4f
#define BUCKET_ACTUATOR_GEAR_RATIO 1.0f
#define BUCKET_ACTUATOR_OUTPUT_RATIO 1.0f
#define BUCKET_ACTUATOR_POLE_PAIRS 1
#define BUCKET_ACTUATOR_TORQUE_CONSTANT 1.0f


nsVescAccess::vesc_param_struct_t central_drive_param =
    {
        .direction = CENTRAL_DRIVE_DIRECTION,
        .max_velocity = MAX_CENTRAL_DRIVE_VELOCITY,
        .max_torque = MAX_CENTRAL_DRIVE_TORQUE,
        .max_duty = MAX_CENTRAL_DRIVE_DUTY,
        .gear_ratio = CENTRAL_DRIVE_GEAR_RATIO,
        .output_ratio = CENTRAL_DRIVE_DIRECTION * CENTRAL_DRIVE_OUTPUT_RATIO,
        .pole_pairs = CENTRAL_DRIVE_POLE_PAIRS,
        .torque_constant = CENTRAL_DRIVE_TORQUE_CONSTANT,
        VESC_CAN_NETWORK,
        .can_id = CENTRAL_DRIVE_ID,
        .name = "central_drive"
    };

nsVescAccess::vesc_param_struct_t bucket_actuator_param =
    {
        .direction = 1.0f,
        .max_velocity = MAX_BUCKET_ACTUATOR_VELOCITY,
        .max_torque = MAX_BUCKET_ACTUATOR_TORQUE,
        .max_duty = MAX_BUCKET_ACTUATOR_DUTY,
        .gear_ratio = BUCKET_ACTUATOR_GEAR_RATIO,
        .output_ratio = BUCKET_ACTUATOR_OUTPUT_RATIO,
        .pole_pairs = BUCKET_ACTUATOR_POLE_PAIRS,
        .torque_constant = BUCKET_ACTUATOR_TORQUE_CONSTANT,
        VESC_CAN_NETWORK,
        .can_id = BUCKET_ACTUATOR_ID,
        .name = "bucket_actuator"
    };

nsVescAccess::vesc_param_struct_t backhoe_actuator_param =
    {
        .direction = BACKHOE_ACTUATOR_DIRECTION,
        .max_velocity = MAX_BACKHOE_ACTUATOR_VELOCITY,
        .max_torque = MAX_BACKHOE_ACTUATOR_TORQUE,
        .max_duty = MAX_LINEAR_DUTY,
        .gear_ratio = BACKHOE_ACTUATOR_GEAR_RATIO,
        .output_ratio = BACKHOE_ACTUATOR_DIRECTION * BACKHOE_ACTUATOR_OUTPUT_RATIO,
        .pole_pairs = BACKHOE_ACTUATOR_POLE_PAIRS,
        .torque_constant = BACKHOE_ACTUATOR_TORQUE_CONSTANT,
        VESC_CAN_NETWORK,
        .can_id = BACKHOE_ACTUATOR_ID,
        .name = "backhoe_actuator"
    };

nsVescAccess::vesc_param_struct_t vibrator_param =
    {
        .direction = 1.0f,
        .max_velocity = MAX_VIBRATOR_VELOCITY,
        .max_torque = MAX_VIBRATOR_TORQUE,
        .max_duty = MAX_VIBRATOR_DUTY,
        .gear_ratio = VIBRATOR_GEAR_RATIO,
        .output_ratio = VIBRATOR_OUTPUT_RATIO,
        .pole_pairs = VIBRATOR_POLE_PAIRS,
        .torque_constant = VIBRATOR_TORQUE_CONSTANT,
        VESC_CAN_NETWORK,
        .can_id = VIBRATOR_ID,
        .name = "vibrator"
    };


#endif //DIG_CONTROL_2_DIG_PARAMS_H
