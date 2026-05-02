/*
 * telem_pack.h — Telemetry and motor-feedback packet assembler
 *
 * Assembles rcu_telem_payload_t and rcu_motor_fb_payload_t from live
 * subsystem data, ready for transmission by eth_udp.
 */
#ifndef TELEM_PACK_H
#define TELEM_PACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rcu_pkt.h"

/**
 * @brief  Fill a slow-telemetry payload from current PDU telem + IMU samples.
 *         Caller provides the destination struct.
 */
void telem_pack_slow(rcu_telem_payload_t *out);

/**
 * @brief  Fill a motor-feedback payload from the current motor_bus cache.
 *         Only slots where feedback has been received are included.
 */
void telem_pack_motor_fb(rcu_motor_fb_payload_t *out);

/**
 * @brief  Decode an inbound motor-command payload and dispatch commands
 *         to motor_bus.
 * @param  entries  Pointer to array of motor command entries.
 * @param  count    Number of entries (payload_len / sizeof(rcu_motor_cmd_entry_t)).
 */
void telem_pack_apply_motor_cmd(const rcu_motor_cmd_entry_t *entries, uint16_t count);

/**
 * @brief  Process an inbound motor supervisory packet.
 *         Handles enable/disable per motor, CSP mode init sequence, and
 *         ctrl_mode selection.  Full software e-stop (enable_mask=0) sends
 *         RS04 Type-4 stop to all currently-enabled motors.
 * @param  sup  Decoded supervisory payload.
 */
void telem_pack_apply_motor_supervisory(const rcu_motor_supervisory_t *sup);

/**
 * @brief  Fill a fast-IMU payload from current IMU samples.
 *         Caller provides the destination struct.
 */
void telem_pack_imu_fast(rcu_imu_fast_t *out);

#ifdef __cplusplus
}
#endif

#endif /* TELEM_PACK_H */
