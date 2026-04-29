/*
 * eth_udp.h — Ethernet UDP transport layer, RCU runtime build
 *
 * Wraps lwIP raw-API UDP sockets.
 *
 * Ports:
 *   7700  RCU → Thor  (slow telem, motor feedback)
 *   7701  Thor → RCU  (motor commands)
 *   7702  RCU → Thor  (supervision events, on-change)
 *
 * Thor IP placeholder: 192.168.100.20  (update once confirmed)
 * RCU static IP:       192.168.100.10  (set in CubeMX lwIP config)
 */
#ifndef ETH_UDP_H
#define ETH_UDP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "rcu_pkt.h"

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise lwIP UDP sockets.
 *         Must be called AFTER MX_LWIP_Init().
 */
void eth_udp_init(void);

/**
 * @brief  Non-blocking poll.  Call every superloop iteration.
 *         Drives MX_LWIP_Process() and drains the Rx socket.
 *         Inbound motor-command packets are dispatched automatically via
 *         telem_pack_apply_motor_cmd().
 * @param  now_ms  Current HAL_GetTick().
 */
void eth_udp_tick(uint32_t now_ms);

/**
 * @brief  Transmit a slow-telemetry packet (type 0x01) on port 7700.
 */
void eth_udp_send_telem(const rcu_telem_payload_t *payload);

/**
 * @brief  Transmit a motor-feedback packet (type 0x02) on port 7700.
 */
void eth_udp_send_motor_fb(const rcu_motor_fb_payload_t *payload);

/**
 * @brief  Transmit a fast-IMU packet (type 0x04) on port 7700.
 *         Called at 200 Hz from the fast-loop scheduling bucket.
 */
void eth_udp_send_imu_fast(const rcu_imu_fast_t *payload);

/**
 * @brief  Transmit a supervision-event packet (type 0x03) on port 7702.
 * @param  event_data  Raw event bytes.
 * @param  len         Length in bytes.
 */
void eth_udp_send_supervision(const uint8_t *event_data, uint16_t len);

/**
 * @brief  Returns true (and clears the flag) if a FORCE_TELEM debug command
 *         was received.  Call from rcu_app.c superloop to trigger immediate TX.
 */
bool eth_udp_consume_force_telem(void);

/**
 * @brief  Returns the number of ETH DMA stalls recovered since boot.
 *         Non-zero means arm/disarm transients triggered the watchdog.
 */
uint32_t eth_udp_get_dma_resets(void);

#ifdef __cplusplus
}
#endif

#endif /* ETH_UDP_H */
