/*
 * motor_bus.h — Motor CAN bus driver for RCU runtime build
 *
 * Drives FDCAN1 (right motor bus) and FDCAN3 (left motor bus) at 1 Mbps.
 * Uses the rs04 codec for frame encoding/decoding.
 *
 * Bus mapping:
 *   FDCAN1 — right motor bus, CAN_MTR_R_STB = PA10 (low = active)
 *   FDCAN3 — left motor bus,  CAN_MTR_L_STB = PD14 (low = active)
 *
 * Motor ID convention:
 *   Right bus: motor IDs 1–8  (configurable on motor hardware)
 *   Left bus:  motor IDs 1–8
 *
 * Operation model (no-RTOS superloop):
 *   - motor_bus_tick() drains both RX FIFOs and caches feedback
 *   - motor_bus_send_cmd() enqueues a command to be sent on next tick
 *   - motor_bus_send_enable() sends an enable/disable frame immediately
 *   - motor_bus_get_feedback() returns the latest feedback for a motor
 */
#ifndef MOTOR_BUS_H
#define MOTOR_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "rs04.h"

/* -----------------------------------------------------------------------
 * Bus index constants
 * ----------------------------------------------------------------------- */
#define MOTOR_BUS_RIGHT  0U
#define MOTOR_BUS_LEFT   1U
#define MOTOR_BUS_COUNT  2U

/* Maximum motors tracked per bus */
#define MOTOR_MAX_PER_BUS  12U

/* -----------------------------------------------------------------------
 * Physical motor ID → CAN bus mapping for PROJ500 12-DOF humanoid.
 * Index = motor_id (1–12).  Index 0 is unused.
 * Update this table if CAN IDs are reassigned on the hardware rig.
 * ----------------------------------------------------------------------- */
static const uint8_t MOTOR_BUS_MAP[MOTOR_MAX_PER_BUS + 1U] = {
    0U,              /* [0]  unused */
    MOTOR_BUS_LEFT,  /* [1]  pelvis_link_l_yaw_joint */
    MOTOR_BUS_RIGHT, /* [2]  pelvis_link_r_yaw_joint */
    MOTOR_BUS_LEFT,  /* [3]  l_hip_yaw_link_l_pitch_joint */
    MOTOR_BUS_RIGHT, /* [4]  r_hip_yaw_link_r_pitch_joint */
    MOTOR_BUS_LEFT,  /* [5]  l_hip_pitch_link_l_roll_joint */
    MOTOR_BUS_RIGHT, /* [6]  r_hip_pitch_link_r_roll_joint */
    MOTOR_BUS_LEFT,  /* [7]  l_thigh_link_l_knee_joint */
    MOTOR_BUS_RIGHT, /* [8]  r_thigh_link_r_knee_joint */
    MOTOR_BUS_LEFT,  /* [9]  l_shank_link_l_ankle_joint */
    MOTOR_BUS_RIGHT, /* [10] r_shank_link_r_ankle_joint */
    MOTOR_BUS_LEFT,  /* [11] l_ankle_link_l_foot_joint */
    MOTOR_BUS_RIGHT, /* [12] r_ankle_link_r_foot_joint */
};

/**
 * @brief  Return the bus index for a given motor_id (1–12).
 *         Returns MOTOR_BUS_COUNT (invalid) if motor_id is out of range.
 */
static inline uint8_t motor_id_to_bus(uint8_t motor_id)
{
    if (motor_id < 1U || motor_id > MOTOR_MAX_PER_BUS) return MOTOR_BUS_COUNT;
    return MOTOR_BUS_MAP[motor_id];
}

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise both motor CAN buses.
 *         Configures FDCAN1 and FDCAN3 with accept-all extended-frame filters.
 *         Enables both CAN transceivers.
 */
void motor_bus_init(void);

/**
 * @brief  Non-blocking poll.  Call every superloop iteration.
 *         Drains RX FIFOs on both buses, updates feedback cache.
 *         Sends any queued commands.
 * @param  now_ms  Current HAL_GetTick().
 */
void motor_bus_tick(uint32_t now_ms);

/**
 * @brief  Queue a control command for a motor.
 *         Sent on the next motor_bus_tick() call.
 * @param  bus       MOTOR_BUS_RIGHT or MOTOR_BUS_LEFT.
 * @param  motor_id  Motor ID (1–8).
 * @param  cmd       Control command.
 */
void motor_bus_send_cmd(uint8_t bus, uint8_t motor_id, const rs04_cmd_t *cmd);

/**
 * @brief  Send a motor enable (Type 3) or stop (Type 4) frame immediately.
 * @param  bus         MOTOR_BUS_RIGHT or MOTOR_BUS_LEFT.
 * @param  motor_id    Motor ID (1–8).
 * @param  enable      true = enable (Type 3), false = stop (Type 4).
 * @param  clear_fault When stopping: set byte[0]=1 to clear faults.
 */
void motor_bus_send_enable(uint8_t bus, uint8_t motor_id,
                           bool enable, bool clear_fault);

/**
 * @brief  Return latest feedback for a motor, or NULL if not yet received.
 * @param  bus       MOTOR_BUS_RIGHT or MOTOR_BUS_LEFT.
 * @param  motor_id  Motor ID (1–12).
 */
const rs04_feedback_t *motor_bus_get_feedback(uint8_t bus, uint8_t motor_id);

/**
 * @brief  Send a Type-18 (comm_type=0x12) single parameter write immediately.
 *         Used to set run_mode, loc_ref, limit_spd etc. for CSP mode.
 * @param  bus       MOTOR_BUS_RIGHT or MOTOR_BUS_LEFT.
 * @param  motor_id  Motor ID (1–12).
 * @param  param_id  Parameter index from the RS04 parameter table (e.g. RS04_PARAM_RUN_MODE).
 * @param  value     Float value to write.
 */
void motor_bus_send_param_write(uint8_t bus, uint8_t motor_id,
                                uint16_t param_id, float value);

/**
 * @brief  Send a Type-6 set-mechanical-zero frame immediately.
 * @param  bus       MOTOR_BUS_RIGHT or MOTOR_BUS_LEFT.
 * @param  motor_id  Motor ID (1–12).
 */
void motor_bus_send_set_zero(uint8_t bus, uint8_t motor_id);

/**
 * @brief  Run internal CAN loopback self-test on both motor buses.
 *         No motors or cables required.  Takes < 25 ms.
 * @return Bitmask: bit0=right OK, bit1=left OK (0x03=both pass).
 *         bit7 set on peripheral re-init failure.
 */
uint8_t motor_bus_loopback_test(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BUS_H */
