/*
 * pdu_mcan_app.c — PDU management CAN application layer, runtime build
 *
 * Peripheral ownership: FDCAN1 (exclusive in PDU_BUILD_MODE_RUNTIME).
 *
 * The st_mcan self-test protocol layer (st_mcan.c) is compiled into both
 * builds but is NEVER CALLED from the runtime path.  The board-provided
 * callback stubs at the bottom of this file satisfy the linker.
 *
 * Filter configuration (4 IDs, two DUAL-mode standard filters):
 *   Filter 0:  ID1=0x500 (RCU status), ID2=0x510 (RCU heartbeat)
 *   Filter 1:  ID1=0x530 (AUX cmd),    ID2=0x531 (CMD_FAULT req)
 * All other frames are rejected at hardware level.
 *
 * Frame layout constants correspond to the plan agreed in architecture review:
 *   0x520 — FPGA status:  [STATUS0, FAULT_CODE, STATE_CODE, ACTIONS]
 *   0x521 — Power rails:  [VRAW_mV_hi, VRAW_mV_lo, V12_mV_hi, V12_mV_lo,
 *                           V24_mV_hi, V24_mV_lo, IVRAW_mA_hi, IVRAW_mA_lo]
 *   0x522 — Currents+T:   [I12_mA_hi, I12_mA_lo, I24_mA_hi, I24_mA_lo,
 *                           T1_dC_hi,  T1_dC_lo,  T2_dC_hi,  T2_dC_lo]
 *   0x523 — SSD:          [I_mA_hi, I_mA_lo, V_10mV_hi, V_10mV_lo,
 *                           P_dW_hi, P_dW_lo, T_dC_hi, T_dC_lo]
 * All multi-byte fields are big-endian (MSB first), int16 unless noted.
 */
#include "pdu_mcan_app.h"
#include "main.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* -----------------------------------------------------------------------
 * Timing
 * ----------------------------------------------------------------------- */
#define MCAN_TX_10HZ_MS   100U   /* 10 Hz telemetry */
#define MCAN_TX_5HZ_MS    200U   /* 5 Hz SSD telemetry */
#define MCAN_HB_MS        500U   /* 2 Hz heartbeat */

/* -----------------------------------------------------------------------
 * Message IDs
 * ----------------------------------------------------------------------- */
#define ID_HB_PDU    0x511U
#define ID_FPGA_STS  0x520U
#define ID_PWR_RAIL  0x521U
#define ID_CURRTEMP  0x522U
#define ID_SSD       0x523U
#define ID_LOCAL_A   0x524U   /* board therms 0-2 + V_source */
#define ID_LOCAL_B   0x525U   /* V_bus + I_coil */

#define ID_RCU_STATUS 0x500U
#define ID_RCU_HB     0x510U
#define ID_CMD_AUX    0x530U
#define ID_CMD_FAULT  0x531U

/* -----------------------------------------------------------------------
 * Peripheral reference (CubeMX-owned)
 * ----------------------------------------------------------------------- */
extern FDCAN_HandleTypeDef hfdcan1;

/* -----------------------------------------------------------------------
 * Module state
 * ----------------------------------------------------------------------- */
static pdu_mcan_cmd_t g_cmds;
static uint32_t       g_next_10hz_ms;
static uint32_t       g_next_5hz_ms;
static uint32_t       g_next_hb_ms;

/* -----------------------------------------------------------------------
 * Private helpers
 * ----------------------------------------------------------------------- */

static void pack_be16(uint8_t *dst, int16_t val)
{
    dst[0] = (uint8_t)((uint16_t)val >> 8);
    dst[1] = (uint8_t)((uint16_t)val & 0xFFU);
}

/* Saturate float to int16 range before packing */
static int16_t fsat_i16(float v)
{
    if (isnan(v)) return 0;
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

static bool fdcan_send(uint32_t id, const uint8_t data[8])
{
    /* TX FIFO is hardware-fixed at 3 slots (SRAMCAN_TFQ_NBR=3 in the G4 HAL).
     * We send up to 5 frames per burst so we must wait for a free slot rather
     * than silently dropping.  At 500 kbit/s a classic 8-byte frame takes
     * ≈130 µs, so a slot is always freed well within 1 ms.
     * The 2 ms timeout guards against CAN bus-off or other fault states. */
    uint32_t t0 = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        if ((HAL_GetTick() - t0) >= 2U) {
            return false;   /* bus fault — skip frame */
        }
    }

    FDCAN_TxHeaderTypeDef txh = { 0 };
    txh.Identifier          = id;
    txh.IdType              = FDCAN_STANDARD_ID;
    txh.TxFrameType         = FDCAN_DATA_FRAME;
    txh.DataLength          = FDCAN_DLC_BYTES_8;
    txh.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txh.BitRateSwitch       = FDCAN_BRS_OFF;
    txh.FDFormat            = FDCAN_CLASSIC_CAN;
    txh.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txh.MessageMarker       = 0U;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txh, data) == HAL_OK;
}

/* -----------------------------------------------------------------------
 * Telemetry packers
 * ----------------------------------------------------------------------- */

static void send_fpga(const fpga_snapshot_t *fpga)
{
    uint8_t d[8] = { 0 };
    if (fpga != NULL && fpga->valid) {
        d[0] = fpga->status0;
        d[1] = fpga->fault_code;
        d[2] = fpga->state_code;
        d[3] = fpga->actions;
        d[4] = fpga->inputs;
        d[5] = fpga->version;
        d[6] = (uint8_t)(fpga->pchg_timer_ms >> 8);
        d[7] = (uint8_t)(fpga->pchg_timer_ms & 0xFFU);
    }
    (void)fdcan_send(ID_FPGA_STS, d);
}

static void send_power_rails(const pdu_ext_adc_t *ext)
{
    uint8_t d[8] = { 0 };
    if (ext != NULL && ext->valid) {
        /* v_vraw packed in 10mV units (×100) to fit int16_t — range 0-65V */
        pack_be16(&d[0], fsat_i16(ext->v_vraw_v    * 100.0f));
        pack_be16(&d[2], fsat_i16(ext->v_12v_v     * 1000.0f));
        pack_be16(&d[4], fsat_i16(ext->v_24v_v     * 1000.0f));
        pack_be16(&d[6], fsat_i16(ext->i_vraw_sw_a * 1000.0f));
    }
    (void)fdcan_send(ID_PWR_RAIL, d);
}

static void send_currents_temps(const pdu_ext_adc_t *ext)
{
    uint8_t d[8] = { 0 };
    if (ext != NULL && ext->valid) {
        pack_be16(&d[0], fsat_i16(ext->i_12v_a  * 1000.0f));
        pack_be16(&d[2], fsat_i16(ext->i_24v_a  * 1000.0f));
        pack_be16(&d[4], fsat_i16(ext->therm1_c * 10.0f));
        pack_be16(&d[6], fsat_i16(ext->therm2_c * 10.0f));
    }
    (void)fdcan_send(ID_CURRTEMP, d);
}

static void send_ssd(const ssd_snapshot_t *ssd)
{
    uint8_t d[8] = { 0 };
    if (ssd != NULL && ssd->valid) {
        pack_be16(&d[0], fsat_i16(ssd->current_a * 1000.0f));
        /* Voltage packed in 10 mV units (×100) — mV units overflow int16
         * at ~32.7 V; the source voltage is ~55 V. Python decodes as /100.0. */
        pack_be16(&d[2], fsat_i16(ssd->voltage_v * 100.0f));
        pack_be16(&d[4], fsat_i16(ssd->current_a * ssd->voltage_v * 10.0f));
        pack_be16(&d[6], fsat_i16(ssd->temp_c    * 10.0f));
    }
    (void)fdcan_send(ID_SSD, d);
}

static void send_local_adc(const pdu_ladc_t *local)
{
    uint8_t a[8] = { 0 };
    uint8_t b[8] = { 0 };
    if (local != NULL && local->valid) {
        pack_be16(&a[0], fsat_i16(local->therm0_c   * 10.0f));
        pack_be16(&a[2], fsat_i16(local->therm1_c   * 10.0f));
        pack_be16(&a[4], fsat_i16(local->therm2_c   * 10.0f));
        pack_be16(&a[6], fsat_i16(local->v_source_v * 100.0f));  /* 10mV units — 53V*100=5300, fits int16 */
        pack_be16(&b[0], fsat_i16(local->v_bus_v    * 100.0f));   /* 10mV units */
        pack_be16(&b[2], fsat_i16(local->i_coil_a   * 1000.0f));
    }
    (void)fdcan_send(ID_LOCAL_A, a);
    (void)fdcan_send(ID_LOCAL_B, b);
}

static void send_heartbeat(void)
{
    uint8_t d[8] = { 0 };
    (void)fdcan_send(ID_HB_PDU, d);
}

/* -----------------------------------------------------------------------
 * RX dispatch
 * ----------------------------------------------------------------------- */

static void handle_rx(uint32_t id, const uint8_t *data, uint32_t dlc)
{
    (void)dlc;
    switch (id) {
    case ID_CMD_AUX:
        g_cmds.aux_cmd_byte    = data[0];
        g_cmds.aux_cmd_pending = true;
        /* Bits 3 and 4 are one-shot trigger bits for buzzer and LED blink */
        if (data[0] & 0x08U) g_cmds.buzz_pending     = true;
        if (data[0] & 0x10U) g_cmds.led_blink_pending = true;
        break;

    case ID_CMD_FAULT:
        if (data[0] & 0x01U) g_cmds.fault_assert_pending = true;
        if (data[0] & 0x02U) g_cmds.fault_clear_pending  = true;
        break;

    case ID_RCU_STATUS:  /* fall through — no action in runtime build */
    case ID_RCU_HB:
    default:
        break;
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void pdu_mcan_app_init(void)
{
    memset(&g_cmds, 0, sizeof(g_cmds));
    g_next_10hz_ms = 0U;
    g_next_5hz_ms  = 0U;
    g_next_hb_ms   = 0U;

    /* Configure CAN transceiver: exit standby */
    HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_RESET);

    /* FDCAN1 was initialised by MX_FDCAN1_Init(); we only add filters + start. */
    FDCAN_FilterTypeDef f = { 0 };

    /* Filter 0: accept RCU status (0x500) and RCU heartbeat (0x510) */
    f.FilterIndex  = 0U;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x500U;   /* base ID */
    f.FilterID2    = 0x7C0U;   /* mask — accepts 0x500–0x53F */
    (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &f);


    /* Reject all unmatched standard frames and all extended frames */
    (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                       FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    (void)HAL_FDCAN_Start(&hfdcan1);
}

/* -----------------------------------------------------------------------
 * Bus-off recovery
 * FDCAN1 enters bus-off when TEC reaches 256 (e.g. from a power-switching
 * transient on the CAN bus during arm/disarm).  The STM32G4 FDCAN does not
 * auto-recover; we must Stop + Start explicitly.
 * ----------------------------------------------------------------------- */
static uint32_t g_busoff_last_ms;

static void check_bus_off(uint32_t now_ms)
{
    /* Rate-limit recovery attempts and log to 1 s */
    if ((now_ms - g_busoff_last_ms) < 1000U) return;

    FDCAN_ProtocolStatusTypeDef psr;
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &psr) != HAL_OK) return;

    if (psr.BusOff) {
        g_busoff_last_ms = now_ms;
        extern void st_dbg_printf(const char *fmt, ...);
        st_dbg_printf("[CAN] FDCAN1 bus-off detected (TEC overflow) — recovering\r\n");
        HAL_FDCAN_Stop(&hfdcan1);
        HAL_FDCAN_Start(&hfdcan1);
        st_dbg_printf("[CAN] FDCAN1 restarted\r\n");
    }
}

void pdu_mcan_app_tick(uint32_t now_ms,
                       const fpga_snapshot_t *fpga,
                       const pdu_ext_adc_t   *ext_adc,
                       const ssd_snapshot_t  *ssd,
                       const pdu_ladc_t      *local_adc)
{
    /* Recover from CAN bus-off before attempting any TX */
    check_bus_off(now_ms);

    /* Drain RX FIFO */
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxd[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxh, rxd) == HAL_OK) {
            handle_rx(rxh.Identifier, rxd, rxh.DataLength);
        }
    }

    /* 2 Hz heartbeat */
    if (now_ms >= g_next_hb_ms) {
        g_next_hb_ms = now_ms + MCAN_HB_MS;
        send_heartbeat();
    }

    /* 10 Hz telemetry */
    if (now_ms >= g_next_10hz_ms) {
        g_next_10hz_ms = now_ms + MCAN_TX_10HZ_MS;
        send_fpga(fpga);
        send_power_rails(ext_adc);
        send_currents_temps(ext_adc);
        send_local_adc(local_adc);
    }

    /* 5 Hz SSD */
    if (now_ms >= g_next_5hz_ms) {
        g_next_5hz_ms = now_ms + MCAN_TX_5HZ_MS;
        send_ssd(ssd);
    }
}

pdu_mcan_cmd_t *pdu_mcan_app_cmds(void)
{
    return &g_cmds;
}

/* -----------------------------------------------------------------------
 * st_mcan.c board callback stubs.
 * These are only compiled in the runtime build.  In the selftest build,
 * pdu_selftest_cli_v3.c provides the real implementations.
 * ----------------------------------------------------------------------- */
#if defined(PDU_BUILD_MODE_RUNTIME)

#include "st_mcan.h"

FDCAN_HandleTypeDef *st_mcan_get_fdcan(void)       { return &hfdcan1; }
void st_mcan_transceiver_set(bool enable)
{
    HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
uint8_t  st_mcan_get_summary_code(void)             { return 0U; }
uint16_t st_mcan_get_status_word(void)              { return 0U; }
uint32_t st_mcan_local_mask_from_selector(uint16_t sel) { (void)sel; return 0U; }
void st_mcan_on_start_remote_selftest(uint32_t mask, bool fast) { (void)mask; (void)fast; }
void st_mcan_action_led_set(bool on)                { (void)on; }
void st_mcan_action_buzzer_set(uint32_t freq_hz)    { (void)freq_hz; }
void st_dbg_printf(const char *fmt, ...)
{
    extern UART_HandleTypeDef huart4;
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)n, 10U);
    }
}

#endif /* PDU_BUILD_MODE_RUNTIME */
