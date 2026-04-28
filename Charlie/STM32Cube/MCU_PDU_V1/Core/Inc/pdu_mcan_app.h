/*
 * pdu_mcan_app.h — PDU management CAN application layer (runtime build)
 *
 * Owns FDCAN1 exclusively in the runtime build.
 * Sends telemetry frames to RCU and receives commands from RCU.
 *
 * TX (PDU→RCU), 10 Hz unless noted:
 *   0x511  Heartbeat         (2 Hz)
 *   0x520  FPGA status       (8B: status0, fault, state, actions,
 *                             inputs, version, pchg_hi, pchg_lo)
 *   0x521  Power rail V      (8B)
 *   0x522  Rail currents+T   (8B)
 *   0x523  SSD readings      (8B, 5 Hz)
 *   0x524  Local ADC A       (8B: therm0, therm1, therm2, V_source)
 *   0x525  Local ADC B       (8B: V_bus, I_coil, spare×4)
 *
 * RX (RCU→PDU):
 *   0x530  AUX switch cmd    (1B, bits[2:0]=ch3/2/1, bit3=buzz trigger,
 *                              bit4=LED-blink trigger)
 *   0x531  CMD_FAULT request (1B, bit0=assert, bit1=clear)
 */
#ifndef PDU_MCAN_APP_H
#define PDU_MCAN_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "fpga_mon.h"
#include "pdu_adc.h"
#include "ssd_energy.h"

/* -----------------------------------------------------------------------
 * Received command latches — read and clear by pdu_app
 * ----------------------------------------------------------------------- */
typedef struct {
    bool     aux_cmd_pending;       /* new AUX command received */
    uint8_t  aux_cmd_byte;          /* bits[2:0]=CH3/CH2/CH1, bit3=buzz, bit4=LED */

    bool     fault_assert_pending;  /* RCU requests CMD_FAULT assert */
    bool     fault_clear_pending;   /* RCU requests CMD_FAULT clear */

    bool     buzz_pending;          /* one-shot: PDU should buzz briefly */
    bool     led_blink_pending;     /* one-shot: PDU should blink LED */
} pdu_mcan_cmd_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise FDCAN1 for runtime operation.
 *         Configures filter to accept IDs 0x500, 0x510, 0x530, 0x531.
 *         Starts FDCAN1 and enables CAN transceiver.
 */
void pdu_mcan_app_init(void);

/**
 * @brief  Non-blocking poll.  Call every superloop iteration.
 *         Drains RX FIFO, dispatches commands, sends telemetry on schedule.
 * @param  now_ms       Current HAL_GetTick().
 * @param  fpga         Latest FPGA snapshot (may be NULL / invalid).
 * @param  ext_adc      Latest external ADC snapshot (may be NULL / invalid).
 * @param  ssd          Latest SSD snapshot (may be NULL / invalid).
 */
void pdu_mcan_app_tick(uint32_t now_ms,
                       const fpga_snapshot_t *fpga,
                       const pdu_ext_adc_t   *ext_adc,
                       const ssd_snapshot_t  *ssd,
                       const pdu_ladc_t      *local_adc);

/**
 * @brief  Return pointer to pending command latch.
 *         Caller should process and clear the pending flags each cycle.
 */
pdu_mcan_cmd_t *pdu_mcan_app_cmds(void);

#ifdef __cplusplus
}
#endif

#endif /* PDU_MCAN_APP_H */
