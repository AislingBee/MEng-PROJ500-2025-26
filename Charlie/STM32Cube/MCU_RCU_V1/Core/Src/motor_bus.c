/*
 * motor_bus.c — Motor CAN bus driver, RCU runtime build
 *
 * Peripheral ownership: FDCAN1 (right bus), FDCAN3 (left bus).
 *
 * Filter configuration for each bus:
 *   One extended-frame mask filter: ID=0, Mask=0 → accept all extended frames.
 *   Standard frames rejected (motors only send extended frames).
 *
 * Command queueing:
 *   A single pending command per motor is stored; overwritten by newer
 *   commands before they are sent.  Motor_bus_tick() sends all pending
 *   commands then clears the queue.
 */
#include "motor_bus.h"
#include "main.h"

#include <string.h>

/* -----------------------------------------------------------------------
 * Peripheral references (CubeMX-owned)
 * ----------------------------------------------------------------------- */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan3;

/* -----------------------------------------------------------------------
 * Per-bus descriptor
 * ----------------------------------------------------------------------- */
typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    GPIO_TypeDef        *stb_port;
    uint16_t             stb_pin;
    rs04_feedback_t      fb[MOTOR_MAX_PER_BUS + 1];  /* index = motor_id */
    bool                 cmd_pending[MOTOR_MAX_PER_BUS + 1];
    rs04_cmd_t           cmd[MOTOR_MAX_PER_BUS + 1];
} bus_t;

static bus_t g_bus[MOTOR_BUS_COUNT] = {
    /* Right bus: FDCAN1 */
    {
        .hfdcan   = &hfdcan1,
        .stb_port = CAN_MTR_R_STB_GPIO_Port,
        .stb_pin  = CAN_MTR_R_STB_Pin,
    },
    /* Left bus: FDCAN3 */
    {
        .hfdcan   = &hfdcan3,
        .stb_port = CAN_MTR_L_STB_GPIO_Port,
        .stb_pin  = CAN_MTR_L_STB_Pin,
    },
};

/* -----------------------------------------------------------------------
 * Helpers
 * ----------------------------------------------------------------------- */

static bool fdcan_send_ext(FDCAN_HandleTypeDef *hfdcan,
                           uint32_t ext_id,
                           const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef txh = { 0 };
    txh.Identifier          = ext_id & 0x1FFFFFFFU;
    txh.IdType              = FDCAN_EXTENDED_ID;
    txh.TxFrameType         = FDCAN_DATA_FRAME;
    txh.DataLength          = FDCAN_DLC_BYTES_8;
    txh.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txh.BitRateSwitch       = FDCAN_BRS_OFF;
    txh.FDFormat            = FDCAN_CLASSIC_CAN;
    txh.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txh.MessageMarker       = 0U;
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txh, data) == HAL_OK;
}

static void init_bus(bus_t *b)
{
    /* Enable transceiver */
    HAL_GPIO_WritePin(b->stb_port, b->stb_pin, GPIO_PIN_RESET);

    /* Extended frame accept-all filter (index 0) */
    FDCAN_FilterTypeDef f = { 0 };
    f.IdType       = FDCAN_EXTENDED_ID;
    f.FilterIndex  = 0U;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0U;   /* match any */
    f.FilterID2    = 0U;   /* mask: all bits don't care */
    (void)HAL_FDCAN_ConfigFilter(b->hfdcan, &f);

    /* Reject standard frames; pass all extended into FIFO0 */
    (void)HAL_FDCAN_ConfigGlobalFilter(b->hfdcan,
                                       FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    (void)HAL_FDCAN_Start(b->hfdcan);
}

static void drain_rx(bus_t *b)
{
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxd[8];
    rs04_feedback_t fb;

    while (HAL_FDCAN_GetRxFifoFillLevel(b->hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(b->hfdcan, FDCAN_RX_FIFO0, &rxh, rxd) != HAL_OK) {
            break;
        }
        if (rxh.IdType != FDCAN_EXTENDED_ID) continue;

        if (rs04_decode_feedback(rxh.Identifier, rxd, &fb)) {
            if (fb.motor_id >= 1U && fb.motor_id <= MOTOR_MAX_PER_BUS) {
                b->fb[fb.motor_id] = fb;
            }
        }
    }
}

static void flush_cmds(bus_t *b)
{
    uint8_t data[8];
    uint32_t ext_id;

    for (uint8_t id = 1U; id <= MOTOR_MAX_PER_BUS; ++id) {
        if (!b->cmd_pending[id]) continue;
        b->cmd_pending[id] = false;
        rs04_encode_cmd(id, &b->cmd[id], &ext_id, data);
        (void)fdcan_send_ext(b->hfdcan, ext_id, data);
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void motor_bus_init(void)
{
    for (uint8_t i = 0U; i < MOTOR_BUS_COUNT; ++i) {
        memset(g_bus[i].fb,          0, sizeof(g_bus[i].fb));
        memset(g_bus[i].cmd_pending, 0, sizeof(g_bus[i].cmd_pending));
        init_bus(&g_bus[i]);
    }
}

void motor_bus_tick(uint32_t now_ms)
{
    (void)now_ms;
    for (uint8_t i = 0U; i < MOTOR_BUS_COUNT; ++i) {
        drain_rx(&g_bus[i]);
        flush_cmds(&g_bus[i]);
    }
}

void motor_bus_send_cmd(uint8_t bus, uint8_t motor_id, const rs04_cmd_t *cmd)
{
    if (bus >= MOTOR_BUS_COUNT) return;
    if (motor_id < 1U || motor_id > MOTOR_MAX_PER_BUS) return;
    g_bus[bus].cmd[motor_id]         = *cmd;
    g_bus[bus].cmd_pending[motor_id] = true;
}

void motor_bus_send_enable(uint8_t bus, uint8_t motor_id,
                           bool enable, bool clear_fault)
{
    if (bus >= MOTOR_BUS_COUNT) return;
    if (motor_id < 1U || motor_id > MOTOR_MAX_PER_BUS) return;

    uint32_t ext_id;
    uint8_t  data[8];
    if (enable) {
        rs04_encode_enable(motor_id, &ext_id, data);
    } else {
        rs04_encode_stop(motor_id, clear_fault, &ext_id, data);
    }
    (void)fdcan_send_ext(g_bus[bus].hfdcan, ext_id, data);
}

const rs04_feedback_t *motor_bus_get_feedback(uint8_t bus, uint8_t motor_id)
{
    if (bus >= MOTOR_BUS_COUNT) return NULL;
    if (motor_id < 1U || motor_id > MOTOR_MAX_PER_BUS) return NULL;
    return &g_bus[bus].fb[motor_id];
}

void motor_bus_send_param_write(uint8_t bus, uint8_t motor_id,
                                uint16_t param_id, float value)
{
    if (bus >= MOTOR_BUS_COUNT) return;
    if (motor_id < 1U || motor_id > MOTOR_MAX_PER_BUS) return;
    uint32_t ext_id;
    uint8_t  data[8];
    rs04_encode_param_write(motor_id, param_id, value, &ext_id, data);
    (void)fdcan_send_ext(g_bus[bus].hfdcan, ext_id, data);
}

void motor_bus_send_set_zero(uint8_t bus, uint8_t motor_id)
{
    if (bus >= MOTOR_BUS_COUNT) return;
    if (motor_id < 1U || motor_id > MOTOR_MAX_PER_BUS) return;
    uint32_t ext_id;
    uint8_t  data[8];
    rs04_encode_set_zero(motor_id, &ext_id, data);
    (void)fdcan_send_ext(g_bus[bus].hfdcan, ext_id, data);
}

/* -----------------------------------------------------------------------
 * CAN loopback test (desk validation, no motors required)
 * Puts each FDCAN peripheral into internal loopback mode via direct CCCR/TEST
 * register access — intentionally avoids HAL_FDCAN_Init() so that message RAM
 * layout and register state of other running FDCAN peripherals (e.g. FDCAN2
 * on the PDU management bus) are not disturbed by a re-init side-effect.
 * Returns bitmask: bit0=right OK, bit1=left OK.
 * ----------------------------------------------------------------------- */

/* Enter loopback, test, restore — returns true if loopback echo verified. */
static bool fdcan_loopback_one(FDCAN_HandleTypeDef *hfdcan)
{
    /* Stop peripheral — sets CCCR.INIT, preserves message RAM completely */
    if (HAL_FDCAN_Stop(hfdcan) != HAL_OK) return false;

    /* Enable CCE (writable only when INIT=1) and test loopback mode.
     * CCCR.TEST=1 unlocks the TEST register; TEST.LBCK=1 routes TX→RX
     * internally while holding the TXD pin recessive (bus not disturbed). */
    SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_CCE);
    SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_TEST);
    SET_BIT(hfdcan->Instance->TEST, FDCAN_TEST_LBCK);

    /* Start — clears CCCR.INIT, sets HAL state to BUSY */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        /* Restore before returning */
        SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_CCE);
        CLEAR_BIT(hfdcan->Instance->TEST, FDCAN_TEST_LBCK);
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_TEST);
        (void)HAL_FDCAN_Start(hfdcan);
        return false;
    }

    /* Send a known test frame — existing accept-all extended filter will
     * route the looped-back copy into RX FIFO 0. */
    const uint8_t tx_data[8] = { 0xDE, 0xAD, 0xBE, 0xEF,
                                  0x01, 0x02, 0x03, 0x04 };
    FDCAN_TxHeaderTypeDef txh = { 0 };
    txh.Identifier          = 0x12345678UL;
    txh.IdType              = FDCAN_EXTENDED_ID;
    txh.TxFrameType         = FDCAN_DATA_FRAME;
    txh.DataLength          = FDCAN_DLC_BYTES_8;
    txh.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txh.BitRateSwitch       = FDCAN_BRS_OFF;
    txh.FDFormat            = FDCAN_CLASSIC_CAN;
    txh.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txh.MessageMarker       = 0U;

    bool ok = false;
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txh, tx_data) == HAL_OK) {
        uint32_t deadline = HAL_GetTick() + 10U;
        while (HAL_GetTick() < deadline) {
            if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
                FDCAN_RxHeaderTypeDef rxh;
                uint8_t rx_data[8];
                if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                           &rxh, rx_data) == HAL_OK) {
                    if (rxh.Identifier == 0x12345678UL &&
                        memcmp(rx_data, tx_data, 8U) == 0) {
                        ok = true;
                    }
                }
                break;
            }
        }
    }

    /* Stop again (CCCR.INIT=1, CCE auto-set by hardware on H7) */
    (void)HAL_FDCAN_Stop(hfdcan);

    /* Disable test mode, restore normal operation */
    SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_CCE);   /* ensure CCE writable */
    CLEAR_BIT(hfdcan->Instance->TEST, FDCAN_TEST_LBCK);
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_TEST);
    (void)HAL_FDCAN_Start(hfdcan);

    /* Flush any stale frames from the RX FIFO */
    {
        FDCAN_RxHeaderTypeDef rxh;
        uint8_t rxd[8];
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
            (void)HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxd);
        }
    }

    return ok;
}

uint8_t motor_bus_loopback_test(void)
{
    uint8_t result = 0U;
    for (uint8_t i = 0U; i < MOTOR_BUS_COUNT; ++i) {
        if (fdcan_loopback_one(g_bus[i].hfdcan)) {
            result |= (1U << i);
        }
    }
    return result;
}
