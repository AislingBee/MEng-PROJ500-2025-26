/*
 * mcan_pdu.h — Management CAN receive/transmit layer for RCU (FDCAN2)
 *
 * RX (PDU→RCU):
 *   0x511  PDU heartbeat
 *   0x520  FPGA status (status0, fault, state, actions, inputs, version, pchg)
 *   0x521  Power rails
 *   0x522  Rail currents + therms
 *   0x523  SSD readings
 *   0x524  Local ADC A (board therm 0-2 + V_source)
 *   0x525  Local ADC B (V_bus, I_coil)
 *
 * TX (RCU→PDU):
 *   0x510  RCU heartbeat (2 Hz)
 *   0x530  AUX switch command (on demand)
 *   0x531  CMD_FAULT request  (on demand)
 */
#ifndef MCAN_PDU_H
#define MCAN_PDU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Cached PDU telemetry (latest received values)
 * ----------------------------------------------------------------------- */
typedef struct {
    /* Validity */
    bool     fpga_valid;
    bool     rails_valid;
    bool     ssd_valid;
    bool     local_valid;         /* 1 = 0x524 received at least once */

    uint32_t fpga_last_ms;
    uint32_t rails_last_ms;
    uint32_t ssd_last_ms;
    uint32_t local_last_ms;
    uint32_t hb_last_ms;          /* last PDU heartbeat received */

    /* FPGA status (0x520) */
    uint8_t  fpga_status0;
    uint8_t  fpga_fault_code;
    uint8_t  fpga_state_code;
    uint8_t  fpga_actions;
    uint8_t  fpga_inputs;         /* register 0x04: estop, arm-permit flags */
    uint8_t  fpga_version;        /* register 0x7F */
    uint16_t fpga_pchg_ms;        /* registers 0x05:0x06 precharge timer */

    /* Power rails (0x521) — v_vraw in 10mV units (int16 × 10 = mV); others in mV/mA */
    int16_t  v_vraw_dv;           /* V_RAW in 10mV units (divide by 100 for V) */
    int16_t  v_12v_mv;
    int16_t  v_24v_mv;
    int16_t  i_vraw_sw_ma;        /* switched VRAW output current [mA] */

    /* Currents + therms (0x522) */
    int16_t  i_12v_ma;
    int16_t  i_24v_ma;
    int16_t  therm1_dc;           /* 0.1 °C units, ext thermistor 1 */
    int16_t  therm2_dc;           /* 0.1 °C units, ext thermistor 2 */

    /* SSD (0x523) */
    int16_t  ssd_i_ma;
    int16_t  ssd_v_dv;            /* V in 10mV units (divide by 100 for V, same scale as v_vraw_dv) */
    int16_t  ssd_p_dw;            /* 0.1 W units */
    int16_t  ssd_t_dc;

    /* Local STM32 ADC (0x524 + 0x525) */
    int16_t  ladc_therm0_dc;      /* board thermistor 0 [0.1 °C] */
    int16_t  ladc_therm1_dc;
    int16_t  ladc_therm2_dc;
    int16_t  ladc_vsource_mv;     /* V_SOURCE [mV] */
    int16_t  ladc_vbus_mv;        /* V_BUS [mV] */
    int16_t  ladc_icoil_ma;       /* I_COIL [mA] */
} pdu_telem_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise FDCAN2 for PDU management CAN operation.
 *         Configures filter to accept 0x511, 0x520–0x523.
 *         Enables CAN_PDU transceiver.
 */
void mcan_pdu_init(void);

/**
 * @brief  Non-blocking poll.  Call every superloop iteration.
 *         Drains RX FIFO and updates the telemetry cache.
 *         Sends heartbeat at 2 Hz.
 * @param  now_ms  Current HAL_GetTick().
 */
void mcan_pdu_tick(uint32_t now_ms);

/**
 * @brief  Return pointer to the latest PDU telemetry cache.
 */
const pdu_telem_t *mcan_pdu_get_telem(void);

/**
 * @brief  Send AUX switch command to PDU.
 * @param  mask  bits[2:0] = CH3/CH2/CH1 desired on/off state.
 */
void mcan_pdu_send_aux_cmd(uint8_t mask);

/**
 * @brief  Send CMD_FAULT request to PDU.
 * @param  assert_fault  true = assert fault; false = clear fault.
 */
void mcan_pdu_send_fault_req(bool assert_fault);

/* st_mcan.c board callback stubs — must be provided here since
 * rcu_selftest_cli_v1.c is excluded in the runtime build.              */
/* (Definitions are in mcan_pdu.c)                                      */

#ifdef __cplusplus
}
#endif

#endif /* MCAN_PDU_H */
