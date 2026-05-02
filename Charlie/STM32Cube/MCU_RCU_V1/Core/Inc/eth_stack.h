/*
 * eth_stack.h
 *
 * Minimal raw Ethernet stack for RCU board (STM32H723, LAN8742A, RMII).
 * Provides: static IP 192.168.100.10, ARP reply, ICMP echo reply, UDP echo.
 *
 * First bring-up milestone — not intended as a long-term networking foundation.
 * lwIP (NO_SYS mode) or another stack is the natural next step.
 */

#ifndef ETH_STACK_H
#define ETH_STACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ======================================================================
 *  Public API
 * ====================================================================== */

/* Bring the Ethernet link up: PHY reset, autoneg, MAC config, HAL_ETH_Start.
 * Blocking — polls PHY registers with HAL_Delay.  Prints result to debug UART.
 * Returns immediately if already up. */
void eth_stack_init(void);

/* Non-blocking receive poll — call from main task loop every iteration.
 * Drains all pending received frames and generates ARP/ICMP/UDP replies. */
void eth_stack_poll(void);

/* Tear down: HAL_ETH_Stop, assert PHY reset. */
void eth_stack_stop(void);

/* Returns true if eth_stack_init() completed successfully and link is up. */
bool eth_stack_is_up(void);

/* Print current IP configuration to debug UART. */
void eth_stack_print_ip(void);

/* Print frame/protocol counters to debug UART. */
void eth_stack_print_stats(void);

/* Enable or disable UDP echo server (port 7777). Default: disabled. */
void eth_stack_udp_echo_set(bool enable);

/* ======================================================================
 *  Stats (readable externally if needed)
 * ====================================================================== */

typedef struct {
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t arp_rx;
    uint32_t arp_tx;
    uint32_t icmp_rx;
    uint32_t icmp_tx;
    uint32_t udp_rx;
    uint32_t udp_tx;
    uint32_t rx_discard;
    uint32_t tx_fail;
} eth_stats_t;

extern eth_stats_t g_eth_stats;

#ifdef __cplusplus
}
#endif

#endif /* ETH_STACK_H */
