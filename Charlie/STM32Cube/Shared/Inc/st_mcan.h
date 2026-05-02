/*
 * st_mcan.h
 *
 * Shared management-CAN protocol layer for PDU/RCU self-test framework.
 *
 * BOARD INTEGRATION
 * =================
 * Each board must provide "st_mcan_config.h" on its include path containing:
 *   - The correct STM32 HAL include (e.g. stm32g4xx_hal.h or stm32h7xx_hal.h)
 *   - #define ST_MCAN_MY_MSG_ID    (PDU: 0x501, RCU: 0x500)
 *   - #define ST_MCAN_PEER_MSG_ID  (PDU: 0x500, RCU: 0x501)
 *   - #define ST_MCAN_MY_HB_ID     (PDU: 0x511, RCU: 0x510)
 *   - #define ST_MCAN_PEER_HB_ID   (PDU: 0x510, RCU: 0x511)
 *
 * The board must also implement and link these extern functions:
 *
 *   FDCAN_HandleTypeDef *st_mcan_get_fdcan(void);
 *   void     st_mcan_transceiver_set(bool enable);
 *   uint8_t  st_mcan_get_summary_code(void);
 *   uint16_t st_mcan_get_status_word(void);
 *   uint32_t st_mcan_local_mask_from_selector(uint16_t sel);
 *   void     st_mcan_on_start_remote_selftest(uint32_t mask, bool fast);
 *   void     st_dbg_printf(const char *fmt, ...);
 */

#ifndef ST_MCAN_H
#define ST_MCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "st_mcan_config.h"   /* board-specific: HAL include + CAN ID defines */
#include "st_common.h"

/* ======================================================================
 *  Board-provided externs (implemented in each board's .c file)
 * ====================================================================== */

extern FDCAN_HandleTypeDef *st_mcan_get_fdcan(void);
extern void     st_mcan_transceiver_set(bool enable);
extern uint8_t  st_mcan_get_summary_code(void);
extern uint16_t st_mcan_get_status_word(void);
extern uint32_t st_mcan_local_mask_from_selector(uint16_t sel);
extern void     st_mcan_on_start_remote_selftest(uint32_t mask, bool fast);
extern void     st_mcan_action_led_set(bool on);
extern void     st_mcan_action_buzzer_set(uint32_t freq_hz);
extern void     st_dbg_printf(const char *fmt, ...);

/* ======================================================================
 *  Public API
 * ====================================================================== */

void st_mcan_init(st_mcan_rt_t *m);
void st_mcan_poll(st_mcan_rt_t *m, uint32_t now_ms);
void st_mcan_clear_stats(st_mcan_rt_t *m);

/* Bus control */
bool st_mcan_bus_config(st_mcan_rt_t *m, bool enable);
void st_mcan_bus_disable(st_mcan_rt_t *m);

/* Low-level send */
bool st_mcan_send_frame(st_mcan_rt_t *m, uint32_t id, const uint8_t payload[8]);
bool st_mcan_send_simple(st_mcan_rt_t *m, uint8_t type, uint8_t seq,
                         uint8_t flags, uint16_t arg0, uint16_t arg1);

/* Heartbeat */
bool st_mcan_send_heartbeat(st_mcan_rt_t *m);

/* Status */
bool st_mcan_send_status_resp(st_mcan_rt_t *m, uint8_t seq);

/* Ping */
void st_mcan_start_ping(st_mcan_rt_t *m, uint32_t count, uint32_t period_ms,
                         bool both, bool report_each);
void st_mcan_stop_ping(st_mcan_rt_t *m);

/* Remote self-test */
bool st_mcan_send_selftest_req(st_mcan_rt_t *m, uint16_t selector,
                                uint16_t flags);
bool st_mcan_send_selftest_result(st_mcan_rt_t *m, uint8_t req_seq,
                                   uint16_t selector, st_status_t status);
bool st_mcan_send_selftest_done(st_mcan_rt_t *m, uint8_t req_seq,
                                 uint16_t fail_count, uint16_t warn_count);
void st_mcan_start_remote_selftest(st_mcan_rt_t *m, uint8_t req_seq,
                                    uint16_t selector, uint16_t flags);

/* Diagnostics / printing */
void st_mcan_print_stats(st_mcan_rt_t *m);
void st_mcan_print_diag(st_mcan_rt_t *m);
void st_mcan_print_timing(st_mcan_rt_t *m);

#ifdef __cplusplus
}
#endif

#endif /* ST_MCAN_H */
