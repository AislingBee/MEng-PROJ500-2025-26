/**
 * Motor Controller Type Definitions and Constants
 * ===============================================
 * 
 * Core data structures and command/parameter definitions
 */

#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>
#include "motor_config.h"

/* ═══════════════════════════════════════════════════════════════════ */
/* CAN COMMAND TYPES */
/* ═══════════════════════════════════════════════════════════════════ */

#define COMM_OPERATION_STATUS     2
#define COMM_ENABLE               3
#define COMM_DISABLE              4
#define COMM_SET_ZERO             6

/* ═══════════════════════════════════════════════════════════════════ */
/* MOTOR PARAMETER IDs */
/* ═══════════════════════════════════════════════════════════════════ */

#define PARAM_MODE                0x7005
#define PARAM_KP                  0x7014   /* Proportional gain for P control */
#define PARAM_KD                  0x7015   /* Damping gain for D control */
#define PARAM_POSITION_TARGET     0x7016
#define PARAM_PP_SPEED_LIMIT      0x7024
#define PARAM_PP_ACCEL            0x7025
#define PARAM_FEEDFORWARD_TORQUE  0x7026   /* Feedforward torque (Nm) */

/* ═══════════════════════════════════════════════════════════════════ */
/* MOTOR CONTROL MODES */
/* ═══════════════════════════════════════════════════════════════════ */

#define MODE_DISABLED             0
#define MODE_POSITION_CONTROL     1
#define MODE_VELOCITY_JOG         7

/* ═══════════════════════════════════════════════════════════════════ */
/* TEMPERATURE SCALING */
/* ═══════════════════════════════════════════════════════════════════ */

#define TEMP_SCALE                0.1f

/* ═══════════════════════════════════════════════════════════════════ */
/* MOTOR STATE STRUCTURE */
/* ═══════════════════════════════════════════════════════════════════ */

/**
 * Complete runtime state of a motor
 * 
 * Combines configuration with current state and setpoints
 */
typedef struct {
    /* Reference to motor configuration */
    const MotorConfig_t *config;
    
    /* Motor identity */
    uint8_t motor_id;
    
    /* Control state */
    uint8_t mode;                 /* MODE_DISABLED, MODE_POSITION_CONTROL, MODE_VELOCITY_JOG */
    uint8_t enabled;              /* 1 = motor active, 0 = motor disabled */
    
    /* Current sensed state (simulated or real) */
    float position;               /* radians */
    float velocity;               /* rad/s */
    float torque;                 /* Nm */
    float temperature;            /* °C */
    
    /* Control setpoints */
    float position_target;        /* radians - target for position control */
    float velocity_target;        /* rad/s - target for velocity jog */
    
    /* Dynamic control parameters (can be set via ROS) */
    float kp;                     /* Proportional gain [0, 500] */
    float kd;                     /* Damping gain [0, 10] */
    float feedforward_torque;     /* Feedforward torque [Nm] */
    
    /* Per-motor control parameters (loaded from config) */
    float pp_speed_limit;         /* rad/s - maximum speed limit */
    float pp_accel;               /* rad/s² - acceleration parameter */
    float position_range;         /* radians - valid range (±value) */
    float velocity_range;         /* rad/s - used for telemetry scaling */
    float torque_range;           /* Nm - used for telemetry scaling */
    
    /* Jog state */
    int32_t jog_direction;        /* -1 = backward, 0 = stopped, 1 = forward */
    
} MotorState_t;

#endif /* MOTOR_TYPES_H */
