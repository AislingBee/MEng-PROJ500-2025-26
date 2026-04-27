/*
 // Motor Controller Header - RobStride Motor Control
 * ==================================================
 * 
 * Public API for motor controller firmware
 */
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "motor_types.h"

/* ═══════════════════════════════════════════════════════════════════ */
/* INITIALIZATION */
/* ═══════════════════════════════════════════════════════════════════ */

/**
 * Initialize motor controller from configuration array
 * Called once at startup
 */
void MotorController_Init(void);

/* ═══════════════════════════════════════════════════════════════════ */
/* MOTOR QUERIES */
/* ═══════════════════════════════════════════════════════════════════ */

/**
 * Get motor state by ID
 * 
 * @param motor_id Motor CAN ID to query
 * @return Pointer to MotorState_t, or NULL if not found
 */
MotorState_t* MotorController_GetMotor(uint8_t motor_id);

/* ═══════════════════════════════════════════════════════════════════ */
/* COMMAND PROCESSING */
/* ═══════════════════════════════════════════════════════════════════ */

/**
 * Process incoming CAN command
 * Decodes motor ID, command type, and parameters
 * 
 * @param ext_id CAN extended identifier
 * @param data 8 bytes of CAN data
 * @param dlc Data length code (0-8)
 */
void MotorController_ProcessCommand(uint32_t ext_id, uint8_t *data, uint8_t dlc);

/* ═══════════════════════════════════════════════════════════════════ */
/* CONTROL & TELEMETRY */
/* ═══════════════════════════════════════════════════════════════════ */

/**
 * Main control loop (called every 1ms via Timer 2)
 * Updates motor states, computes control laws, sends telemetry
 */
void MotorController_ControlLoop(void);

#endif /* MOTOR_CONTROLLER_H */
