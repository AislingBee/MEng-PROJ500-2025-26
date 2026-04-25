/**
 * Motor Configuration Header
 * ==========================
 * 
 * Defines motor parameter structures and default configurations.
 * Add new motors by creating entries in the MOTOR_CONFIGS array.
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <stdint.h>
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════ */
/* MOTOR CONFIGURATION STRUCTURE */
/* ═══════════════════════════════════════════════════════════════════ */

typedef struct {
    /* Motor Identity */
    uint8_t motor_id;
    
    /* Control Parameters */
    float pp_speed_limit;      /* rad/s - max speed during point-to-point */
    float pp_accel;            /* rad/s² - acceleration limit */
    float position_range;      /* radians - valid motion range (±value) */
    float velocity_range;      /* rad/s - max velocity */
    float torque_range;        /* Nm - max torque (for telemetry scaling) */
    
    /* Initial State */
    uint8_t enabled_on_startup;  /* 1 = auto-enable, 0 = start disabled */
    float initial_position;      /* radians - starting position */
    float initial_velocity;      /* rad/s - starting velocity */
    
} MotorConfig_t;

/* ═══════════════════════════════════════════════════════════════════ */
/* PREDEFINED MOTOR CONFIGURATIONS */
/* ═══════════════════════════════════════════════════════════════════ */

/**
 * Default configuration for Motor 1 (ID = 127)
 * RobStride motor with standard settings
 */
static const MotorConfig_t MOTOR1_CONFIG = {
    .motor_id = 127,
    .pp_speed_limit = 10.0f,          /* 10 rad/s */
    .pp_accel = 10.0f,                /* 10 rad/s² */
    .position_range = 2.0f * M_PI,    /* ±2π radians (±360°) */
    .velocity_range = 15.0f,          /* ±15 rad/s */
    .torque_range = 120.0f,           /* ±120 Nm */
    .enabled_on_startup = 0,          /* Start disabled */
    .initial_position = 0.0f,         /* At zero */
    .initial_velocity = 0.0f,         /* Not moving */
};

/**
 * Default configuration for Motor 2 (ID = 1)
 * RobStride motor with standard settings
 */
static const MotorConfig_t MOTOR2_CONFIG = {
    .motor_id = 1,
    .pp_speed_limit = 10.0f,
    .pp_accel = 10.0f,
    .position_range = 2.0f * M_PI,
    .velocity_range = 15.0f,
    .torque_range = 120.0f,
    .enabled_on_startup = 0,
    .initial_position = 0.0f,
    .initial_velocity = 0.0f,
};

/**
 * Array of all motor configurations
 * Each motor in your system should have an entry here
 */
static const MotorConfig_t MOTOR_CONFIGS[] = {
    MOTOR1_CONFIG,
    MOTOR2_CONFIG,
    /* Add more motors here:
       {
           .motor_id = 2,
           .pp_speed_limit = 10.0f,
           .pp_accel = 10.0f,
           .position_range = 2.0f * M_PI,
           .velocity_range = 15.0f,
           .torque_range = 120.0f,
           .enabled_on_startup = 0,
           .initial_position = 0.0f,
           .initial_velocity = 0.0f,
       },
    */
};

#define NUM_MOTOR_CONFIGS  (sizeof(MOTOR_CONFIGS) / sizeof(MOTOR_CONFIGS[0]))

/* ═══════════════════════════════════════════════════════════════════ */
/* GLOBAL SYSTEM PARAMETERS */
/* ═══════════════════════════════════════════════════════════════════ */

/** Host ID used in CAN frames */
#define HOST_ID 0xFD

/** Telemetry broadcast period in milliseconds */
#define TELEMETRY_PERIOD_MS 50

/** Control loop frequency in Hz */
#define CONTROL_LOOP_FREQ_HZ 1000

/* ═══════════════════════════════════════════════════════════════════ */
/* HELPER FUNCTIONS */
/* ═══════════════════════════════════════════════════════════════════ */

/**
 * Get motor configuration by motor ID
 * 
 * @param motor_id The ID of the motor to look up
 * @return Pointer to motor configuration, or NULL if not found
 */
static inline const MotorConfig_t* MotorConfig_Get(uint8_t motor_id) {
    for (uint32_t i = 0; i < NUM_MOTOR_CONFIGS; i++) {
        if (MOTOR_CONFIGS[i].motor_id == motor_id) {
            return &MOTOR_CONFIGS[i];
        }
    }
    return NULL;
}

/**
 * Get number of configured motors
 */
static inline uint32_t MotorConfig_GetCount(void) {
    return NUM_MOTOR_CONFIGS;
}

#endif /* MOTOR_CONFIG_H */
