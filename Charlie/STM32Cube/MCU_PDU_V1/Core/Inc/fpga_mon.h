/*
 * fpga_mon.h — FPGA I2C monitor for PDU runtime build
 *
 * Polls the MachXO2 FPGA via I2C4 at a fixed interval and caches its
 * register state.  All access is from the superloop only; no ISR context.
 *
 * FPGA I2C address : 0x55 (7-bit)
 * Register map     : 0x00 STATUS0, 0x01 FAULT_CODE, 0x02 STATE_CODE,
 *                    0x03 ACTIONS, 0x04 INPUTS, 0x05 PCHG_HI, 0x06 PCHG_LO,
 *                    0x7F VERSION
 */
#ifndef FPGA_MON_H
#define FPGA_MON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * FPGA state codes (STATE_CODE register 0x02)
 * ----------------------------------------------------------------------- */
#define FPGA_STATE_IDLE       0U
#define FPGA_STATE_PRECHARGE  1U
#define FPGA_STATE_ARMED      2U
#define FPGA_STATE_COMPUTE    3U

/* -----------------------------------------------------------------------
 * Snapshot of all FPGA registers, populated every FPGA_MON_INTERVAL_MS.
 * Treat as read-only from outside this module.
 * ----------------------------------------------------------------------- */
typedef struct {
    bool     valid;           /* true once first successful read */
    uint32_t last_read_ms;

    /* Raw register bytes */
    uint8_t  status0;         /* 0x00 */
    uint8_t  fault_code;      /* 0x01 */
    uint8_t  state_code;      /* 0x02  0=IDLE,1=PRECHARGE,2=ARMED,3=COMPUTE */
    uint8_t  actions;         /* 0x03 */
    uint8_t  inputs;          /* 0x04 */
    uint16_t pchg_timer_ms;   /* 0x05:0x06 big-endian */
    uint8_t  version;         /* 0x7F */

    /* Decoded convenience flags from STATUS0 */
    bool fault_latch;
    bool precharge_latch;
    bool motor_en;
    bool compute_en;
    bool ovuv_ok;
    bool arm_permit;

    /* Decoded from INPUTS */
    bool estop_ok;
    bool mcu_cmd_arm_seen;
    bool remote_arm_latch;
} fpga_snapshot_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise the FPGA monitor (no HAL calls yet).
 *         Must be called once before fpga_mon_tick().
 */
void fpga_mon_init(void);

/**
 * @brief  Non-blocking poll.  Call every superloop iteration.
 *         Performs an I2C transaction at most every FPGA_MON_INTERVAL_MS.
 * @param  now_ms  Current HAL_GetTick() value.
 */
void fpga_mon_tick(uint32_t now_ms);

/**
 * @brief  Return a pointer to the most-recent FPGA snapshot.
 *         Check snapshot->valid before trusting the data.
 */
const fpga_snapshot_t *fpga_mon_get(void);

#ifdef __cplusplus
}
#endif

#endif /* FPGA_MON_H */
