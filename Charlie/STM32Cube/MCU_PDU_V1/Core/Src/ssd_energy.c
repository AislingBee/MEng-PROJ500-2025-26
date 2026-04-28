/*
 * ssd_energy.c — SSD RS485 energy meter driver, PDU runtime build
 *
 * UART configuration: USART3, 19200-8N1, DE pin = RS485_DE_GPIO_Port/Pin.
 * The UART MspInit configures DE as AF7; we override it to GPIO output for
 * manual direction control (same technique as pdu_selftest_cli_v3.c).
 */
#include "ssd_energy.h"
#include "main.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* -----------------------------------------------------------------------
 * Timing
 * ----------------------------------------------------------------------- */
#define SSD_ENERGY_INTERVAL_MS   200U   /* 5 Hz */
#define SSD_UART_BAUD            19200U
#define SSD_UART_TIMEOUT_MS      800U
#define SSD_RX_BUF_LEN           64U
#define SSD_RAW_TIMEOUT_MS       1000U  /* match selftest ST_SSD_RAW_TIMEOUT_MS */
#define SSD_INTER_CMD_DELAY_MS   20U

/* -----------------------------------------------------------------------
 * Peripheral references (CubeMX-owned)
 * ----------------------------------------------------------------------- */
extern UART_HandleTypeDef huart3;
extern void st_dbg_printf(const char *fmt, ...);

/* -----------------------------------------------------------------------
 * Module state
 * ----------------------------------------------------------------------- */
static ssd_snapshot_t g_snap;
static uint32_t       g_next_ms;
static bool           g_logged;

/* -----------------------------------------------------------------------
 * Direction control
 * ----------------------------------------------------------------------- */
static inline void de_set(bool tx_enable)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin,
                      tx_enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* -----------------------------------------------------------------------
 * UART re-apply — mirrors st_ssd_uart_apply_defaults() from selftest.
 * Called at the top of every tick so that any UART error state accumulated
 * during the previous tick (e.g. framing error from bus noise at power-on)
 * is cleared before the next set of queries.  The selftest does the same
 * DeInit→Init→GPIO-override before every st_ssd_cache_refresh() call.
 * ----------------------------------------------------------------------- */
static void reapply_uart(void)
{
    (void)HAL_UART_DeInit(&huart3);

    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = SSD_UART_BAUD;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    (void)HAL_UART_Init(&huart3);
    (void)HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_DisableFifoMode(&huart3);

    GPIO_InitTypeDef gpio_de = { 0 };
    gpio_de.Pin   = RS485_DE_Pin;
    gpio_de.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_de.Pull  = GPIO_NOPULL;
    gpio_de.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS485_DE_GPIO_Port, &gpio_de);
    de_set(false);
}

/* -----------------------------------------------------------------------
 * UART helpers — extracted from pdu_selftest_cli_v3.c patterns
 * ----------------------------------------------------------------------- */
static void flush_rx(void)
{
    uint32_t t0 = HAL_GetTick();
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    while ((HAL_GetTick() - t0) < 20U) {
        if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET) {
            volatile uint8_t dump = (uint8_t)(huart3.Instance->RDR & 0xFFU);
            (void)dump;
            t0 = HAL_GetTick();
        }
    }
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
}

static void post_tx_cleanup(void)
{
    HAL_Delay(1U);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET) {
        (void)(huart3.Instance->RDR & 0xFFU);
    }
}

static bool send_cmd(const char *cmd)
{
    flush_rx();
    de_set(true);
    HAL_Delay(2U);

    HAL_StatusTypeDef st = HAL_UART_Transmit(&huart3,
                                              (uint8_t *)cmd,
                                              (uint16_t)strlen(cmd),
                                              SSD_UART_TIMEOUT_MS);
    if (st != HAL_OK) {
        de_set(false);
        return false;
    }

    /* Wait for TC (last bit shifted out) before dropping DE — with timeout
     * to prevent infinite hang if UART peripheral enters a bad state. */
    uint32_t tc_t0 = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - tc_t0) > 100U) break;
    }
    de_set(false);
    post_tx_cleanup();
    return true;
}

static size_t read_capture(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    size_t idx = 0U;
    uint8_t ch = 0U;
    uint32_t t0 = HAL_GetTick();
    bool got_any = false;

    while ((HAL_GetTick() - t0) < timeout_ms && idx < len) {
        HAL_StatusTypeDef rc = HAL_UART_Receive(&huart3, &ch, 1U, 20U);
        if (rc == HAL_OK) {
            buf[idx++] = ch;
            got_any = true;
            t0 = HAL_GetTick();
        } else {
            if (rc == HAL_ERROR) {
                __HAL_UART_CLEAR_OREFLAG(&huart3);
                __HAL_UART_CLEAR_NEFLAG(&huart3);
                __HAL_UART_CLEAR_FEFLAG(&huart3);
                __HAL_UART_CLEAR_PEFLAG(&huart3);
            }
            if (got_any && (HAL_GetTick() - t0) > 50U) {
                break;
            }
        }
    }
    return idx;
}

/* Extract the last non-empty line from raw bytes */
static bool extract_last_line(const uint8_t *raw, size_t raw_len,
                               char *line, size_t line_len)
{
    if (line == NULL || line_len == 0U) return false;
    line[0] = '\0';

    size_t best_start = 0U, best_len = 0U;
    size_t cur_start  = 0U, cur_len  = 0U;

    for (size_t i = 0U; i < raw_len; ++i) {
        uint8_t ch = raw[i];
        if (ch == '\r' || ch == '\n') {
            if (cur_len > 0U) {
                best_start = cur_start;
                best_len   = cur_len;
                cur_len    = 0U;
            }
            cur_start = i + 1U;
        } else {
            if (cur_len == 0U) cur_start = i;
            cur_len++;
        }
    }
    if (cur_len > 0U) { best_start = cur_start; best_len = cur_len; }
    if (best_len == 0U) return false;
    if (best_len >= line_len) best_len = line_len - 1U;
    memcpy(line, &raw[best_start], best_len);
    line[best_len] = '\0';
    return true;
}

/* Query command, parse signed integer value after expected prefix char.
 * e.g. "A1234" → prefix='A', value=1234 */
static bool query_i32(const char *cmd, char expected_prefix, int32_t *value)
{
    uint8_t raw[SSD_RX_BUF_LEN];
    char    line[SSD_RX_BUF_LEN];
    char    num[24];
    size_t  ni = 0U;
    bool    started = false;

    if (!send_cmd(cmd)) return false;

    size_t n = read_capture(raw, sizeof(raw), SSD_RAW_TIMEOUT_MS);
    if (n == 0U) return false;
    if (!extract_last_line(raw, n, line, sizeof(line))) return false;
    if (line[0] != expected_prefix) return false;

    for (size_t i = 1U; line[i] != '\0' && ni < (sizeof(num) - 1U); ++i) {
        char c = line[i];
        if (!started) {
            if (c == '-' || c == '+' || isdigit((unsigned char)c)) {
                num[ni++] = c;
                started = true;
            }
        } else {
            if (isdigit((unsigned char)c)) num[ni++] = c;
            else break;
        }
    }
    num[ni] = '\0';
    if (ni == 0U) return false;
    *value = (int32_t)strtol(num, NULL, 10);
    return true;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void ssd_energy_init(void)
{
    memset(&g_snap, 0, sizeof(g_snap));
    g_next_ms = 0U;
    g_logged  = false;

    /* DeInit first to reset handle state: CubeMX called HAL_RS485Ex_Init which
     * leaves DEM=1 and gState=READY. DeInit→Init sequence matches the selftest's
     * st_ssd_uart_apply() which does the same and is known to work. */
    (void)HAL_UART_DeInit(&huart3);

    /* Apply 19200-8N1 and override DE pin to GPIO output */
    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = SSD_UART_BAUD;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    (void)HAL_UART_Init(&huart3);
    (void)HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_DisableFifoMode(&huart3);

    /* Override DE pin from AF7 to GPIO output so de_set() works */
    GPIO_InitTypeDef gpio_de  = { 0 };
    gpio_de.Pin   = RS485_DE_Pin;
    gpio_de.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_de.Pull  = GPIO_NOPULL;
    gpio_de.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS485_DE_GPIO_Port, &gpio_de);
    de_set(false);
}

void ssd_energy_tick(uint32_t now_ms)
{
    if (now_ms < g_next_ms) {
        return;
    }
    g_next_ms = now_ms + SSD_ENERGY_INTERVAL_MS;

    /* Re-apply UART settings on every tick — mirrors st_ssd_uart_apply_defaults()
     * in the selftest, which does the same before every cache refresh.
     * This clears any UART error state (e.g. framing error from bus noise) that
     * would otherwise cause all subsequent HAL_UART_Receive calls to return
     * HAL_ERROR and make query_i32 fail permanently. */
    reapply_uart();

    int32_t current_ma = 0;
    int32_t vbus_mv    = 0;
    int32_t temp_dc    = 0;

    bool ok  = query_i32(":1GA\r", 'A', &current_ma);
    HAL_Delay(SSD_INTER_CMD_DELAY_MS);
    ok = ok && query_i32(":1GV\r", 'V', &vbus_mv);
    HAL_Delay(SSD_INTER_CMD_DELAY_MS);
    ok = ok && query_i32(":1GT\r", 'T', &temp_dc);

    if (ok) {
        g_snap.current_a  = (float)current_ma / 1000.0f;
        /* Meter may return negative voltage depending on connection polarity;
         * we always want the magnitude. */
        g_snap.voltage_v  = fabsf((float)vbus_mv / 1000.0f);
        g_snap.temp_c     = (float)temp_dc    / 10.0f;
        g_snap.last_ok_ms = HAL_GetTick();
        g_snap.valid      = true;
        if (!g_logged) {
            g_logged = true;
            st_dbg_printf("[SSD] first ok: I=%d mA V=%d mV T=%d dC\r\n",
                          (int)current_ma, (int)vbus_mv, (int)temp_dc);
        }
    } else {
        if (!g_logged) {
            g_logged = true;
            st_dbg_printf("[SSD] query failed (GA ok=%d)\r\n", (int)(current_ma != 0));
        }
    }
}

const ssd_snapshot_t *ssd_energy_get(void)
{
    return &g_snap;
}
