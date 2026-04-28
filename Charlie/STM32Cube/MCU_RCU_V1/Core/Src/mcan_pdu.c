/*
 * mcan_pdu.c — Management CAN layer, RCU runtime build (FDCAN2)
 *
 * Peripheral ownership: FDCAN2 (exclusive in RCU_BUILD_MODE_RUNTIME).
 *
 * FDCAN2 on STM32H723 has StdFiltersNbr=1 in the .ioc, so only 1 filter
 * slot is available.  Single mask-mode filter:
 *   ID=0x500, Mask=0x7C0 → compares bits[10:6] only → accepts 0x500–0x53F
 *   (0x7E0 was wrong: it compared bits[10:5], rejecting 0x520–0x52F)
 *
 * Accepted frames dispatched in handle_rx():
 *   0x511  PDU heartbeat
 *   0x520  FPGA status (status0, fault_code, state_code, actions,
 *                       inputs, version, pchg_timer)
 *   0x521  Power rails  (V_vraw, V_12v, V_24v, I_vraw)
 *   0x522  Currents+T   (I_12v, I_24v, therm1, therm2)
 *   0x523  SSD          (I, V, P, T)
 *   0x524  Local ADC A  (board therm0/1/2, V_source)
 *   0x525  Local ADC B  (V_bus, I_coil)
 */
#include "mcan_pdu.h"
#include "main.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* -----------------------------------------------------------------------
 * Timing
 * ----------------------------------------------------------------------- */
#define MCAN_PDU_HB_MS   500U   /* 2 Hz heartbeat */

/* -----------------------------------------------------------------------
 * Message IDs
 * ----------------------------------------------------------------------- */
#define ID_PDU_HB        0x511U
#define ID_FPGA_STS      0x520U
#define ID_PWR_RAILS     0x521U
#define ID_CURR_TEMP     0x522U
#define ID_SSD           0x523U
#define ID_LOCAL_ADC_A   0x524U
#define ID_LOCAL_ADC_B   0x525U

#define ID_RCU_HB        0x510U
#define ID_CMD_AUX       0x530U
#define ID_CMD_FAULT     0x531U

/* -----------------------------------------------------------------------
 * Peripheral reference
 * ----------------------------------------------------------------------- */
extern FDCAN_HandleTypeDef hfdcan2;

/* -----------------------------------------------------------------------
 * Module state
 * ----------------------------------------------------------------------- */
static pdu_telem_t g_telem;
static uint32_t    g_next_hb_ms;

/* -----------------------------------------------------------------------
 * Private helpers
 * ----------------------------------------------------------------------- */

static int16_t unpack_be16(const uint8_t *src)
{
    return (int16_t)(((uint16_t)src[0] << 8) | src[1]);
}

static bool fdcan2_send(uint32_t id, const uint8_t data[8])
{
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
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txh, data) == HAL_OK;
}

static void handle_rx(uint32_t id, const uint8_t *d)
{
    uint32_t now = HAL_GetTick();
    switch (id) {
    case ID_PDU_HB:
        g_telem.hb_last_ms = now;
        break;

    case ID_FPGA_STS:
        g_telem.fpga_status0    = d[0];
        g_telem.fpga_fault_code = d[1];
        g_telem.fpga_state_code = d[2];
        g_telem.fpga_actions    = d[3];
        g_telem.fpga_inputs     = d[4];
        g_telem.fpga_version    = d[5];
        g_telem.fpga_pchg_ms    = (uint16_t)(((uint16_t)d[6] << 8) | d[7]);
        g_telem.fpga_last_ms    = now;
        g_telem.fpga_valid      = true;
        break;

    case ID_PWR_RAILS:
        g_telem.v_vraw_dv    = unpack_be16(&d[0]);   /* 10mV units */
        g_telem.v_12v_mv     = unpack_be16(&d[2]);
        g_telem.v_24v_mv     = unpack_be16(&d[4]);
        g_telem.i_vraw_sw_ma = unpack_be16(&d[6]);
        g_telem.rails_last_ms = now;
        g_telem.rails_valid   = true;
        break;

    case ID_CURR_TEMP:
        g_telem.i_12v_ma    = unpack_be16(&d[0]);
        g_telem.i_24v_ma    = unpack_be16(&d[2]);
        g_telem.therm1_dc   = unpack_be16(&d[4]);
        g_telem.therm2_dc   = unpack_be16(&d[6]);
        break;

    case ID_SSD:
        g_telem.ssd_i_ma    = unpack_be16(&d[0]);
        g_telem.ssd_v_dv    = unpack_be16(&d[2]);
        g_telem.ssd_p_dw    = unpack_be16(&d[4]);
        g_telem.ssd_t_dc    = unpack_be16(&d[6]);
        g_telem.ssd_last_ms = now;
        g_telem.ssd_valid   = true;
        break;

    case ID_LOCAL_ADC_A:
        g_telem.ladc_therm0_dc  = unpack_be16(&d[0]);
        g_telem.ladc_therm1_dc  = unpack_be16(&d[2]);
        g_telem.ladc_therm2_dc  = unpack_be16(&d[4]);
        g_telem.ladc_vsource_mv = unpack_be16(&d[6]);
        g_telem.local_last_ms   = now;
        g_telem.local_valid     = true;
        break;

    case ID_LOCAL_ADC_B:
        g_telem.ladc_vbus_mv  = unpack_be16(&d[0]);
        g_telem.ladc_icoil_ma = unpack_be16(&d[2]);
        break;

    default:
        break;
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void mcan_pdu_init(void)
{
    memset(&g_telem, 0, sizeof(g_telem));
    g_next_hb_ms = 0U;

    /* Enable CAN_PDU transceiver (active low standby) */
    HAL_GPIO_WritePin(CAN_PDU_STB_GPIO_Port, CAN_PDU_STB_Pin, GPIO_PIN_RESET);

    /* Mask filter: base ID=0x500, mask=0x7C0 → compares bits[10:6] only,
     * accepts 0x500–0x53F.  Previous mask 0x7E0 compared bits[10:5] and
     * incorrectly rejected 0x520–0x52F (bit5=1). */
    FDCAN_FilterTypeDef f = { 0 };
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0U;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x500U;  /* base ID */
    f.FilterID2    = 0x7C0U;  /* mask: compare bits[10:6] → accepts 0x500–0x53F */
    (void)HAL_FDCAN_ConfigFilter(&hfdcan2, &f);

    (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                       FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    (void)HAL_FDCAN_Start(&hfdcan2);
}

void mcan_pdu_tick(uint32_t now_ms)
{
    /* Drain RX FIFO */
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxd[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rxh, rxd) == HAL_OK) {
            handle_rx(rxh.Identifier, rxd);
        }
    }

    /* 2 Hz heartbeat */
    if (now_ms >= g_next_hb_ms) {
        g_next_hb_ms = now_ms + MCAN_PDU_HB_MS;
        uint8_t hb[8] = { 0 };
        (void)fdcan2_send(ID_RCU_HB, hb);
    }
}

const pdu_telem_t *mcan_pdu_get_telem(void)
{
    return &g_telem;
}

void mcan_pdu_send_aux_cmd(uint8_t mask)
{
    uint8_t d[8] = { 0 };
    d[0] = mask;   /* full mask — bit3=buzz, bit4=LED must not be stripped */
    (void)fdcan2_send(ID_CMD_AUX, d);
}

void mcan_pdu_send_fault_req(bool assert_fault)
{
    uint8_t d[8] = { 0 };
    d[0] = assert_fault ? 0x01U : 0x02U;
    (void)fdcan2_send(ID_CMD_FAULT, d);
}

/* -----------------------------------------------------------------------
 * st_mcan.c board callback stubs.
 * Only compiled in the runtime build; rcu_selftest_cli_v1.c provides
 * the real implementations in the selftest build.
 * ----------------------------------------------------------------------- */
#if defined(RCU_BUILD_MODE_RUNTIME)

#include "st_mcan.h"

FDCAN_HandleTypeDef *st_mcan_get_fdcan(void)        { return &hfdcan2; }
void st_mcan_transceiver_set(bool enable)
{
    HAL_GPIO_WritePin(CAN_PDU_STB_GPIO_Port, CAN_PDU_STB_Pin,
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
    extern UART_HandleTypeDef huart2;
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 10U);
    }
}

#endif /* RCU_BUILD_MODE_RUNTIME */
