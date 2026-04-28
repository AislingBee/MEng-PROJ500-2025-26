
/* In RCU_BUILD_MODE_RUNTIME all symbols in this file are provided by the
 * runtime modules (mcan_pdu.c, etc.).  Exclude the whole translation unit. */
#if !defined(RCU_BUILD_MODE_RUNTIME)

#include "rcu_selftest_cli_v1.h"
#include "main.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "st_common.h"
#include "st_mcan.h"
#include "eth_stack.h"

extern ADC_HandleTypeDef hadc1;
extern ETH_HandleTypeDef heth;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern I2C_HandleTypeDef hi2c3;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart2;

#define ST_DBG_UART          huart2
#define ST_EXP_UART          huart4
#define ST_ESP_UART          huart8
#define ST_CAN_PDU           hfdcan2
#define ST_CAN_RIGHT         hfdcan1
#define ST_CAN_LEFT          hfdcan3
#define ST_ETH               heth
#define ST_THERM_ADC         hadc1
#define ST_EXP_I2C           hi2c3
#define ST_IMU0_SPI          hspi4
#define ST_IMU1_SPI          hspi3
#define ST_BUZZER_TIM        htim1

#define ST_VERSION                    "rcu-selftest-cli-0.1"
#define ST_CLI_BUF_LEN                128U
#define ST_DEFAULT_LOOP_DELAY_MS      1000U
#define ST_FAST_LOOP_DELAY_MS         50U
#define ST_DEFAULT_MONITOR_MS         1000U

#define ST_TEST_GPIO                  (1UL << 0)
#define ST_TEST_BUZZER                (1UL << 1)
#define ST_TEST_THERM                 (1UL << 2)
#define ST_TEST_PDUFAULT              (1UL << 3)
#define ST_TEST_CAN_PDU               (1UL << 4)
#define ST_TEST_CAN_LEFT              (1UL << 5)
#define ST_TEST_CAN_RIGHT             (1UL << 6)
#define ST_TEST_ETH                   (1UL << 7)
#define ST_TEST_IMU0                  (1UL << 8)
#define ST_TEST_IMU1                  (1UL << 9)
#define ST_TEST_ESP                   (1UL << 10)
#define ST_TEST_EXP                   (1UL << 11)
#define ST_TEST_MCAN                  (1UL << 12)
#define ST_TEST_ALL                   ((1UL << 13) - 1U)

#define ST_LED_ACTIVE_LOW             0
#define ST_THERM_SERIES_R             10000.0f
#define ST_THERM_BETA                 3435.0f
#define ST_THERM_R25                  10000.0f
#define ST_VREF                       3.3f

#define ST_IMU_WHOAMI_REG             0x0FU
#define ST_LSM6DSOX_WHOAMI            0x6CU
#define ST_IMU_CTRL1_XL               0x10U
#define ST_IMU_CTRL2_G                0x11U
#define ST_IMU_CTRL3_C                0x12U
#define ST_IMU_OUT_TEMP_L             0x20U
#define ST_IMU_OUTX_L_G               0x22U
#define ST_IMU_OUTX_L_A               0x28U

#define ST_ETH_PHY_BMCR               0x00U
#define ST_ETH_PHY_BMSR               0x01U
#define ST_ETH_PHY_ID1                0x02U
#define ST_ETH_PHY_ID2                0x03U
#define ST_ETH_BMSR_LINK              0x0004U
#define ST_ETH_BMSR_AUTONEGCOMP       0x0020U


typedef struct {
    int16_t temp_raw;
    int16_t gyro[3];
    int16_t accel[3];
} st_imu_sample_t;

static void st_prompt(void);
static void st_handle_uart_rx(void);
static void st_process_line(char *line);
static void st_runner_poll(uint32_t now_ms);
static void st_monitor_poll(uint32_t now_ms);
static void st_stream_poll(uint32_t now_ms);
static void st_print_help(void);
static void st_print_tests(void);
static void st_print_status(void);
static void st_print_summary(void);
static int st_test_index_from_name(const char *name);
static void st_start_all(void);
static void st_start_one(uint32_t mask);
static void st_abort(bool announce);
static void st_reset_all_tests(void);

uint8_t st_mcan_get_summary_code(void);
uint16_t st_mcan_get_status_word(void);
uint32_t st_mcan_local_mask_from_selector(uint16_t selector);
void st_mcan_on_start_remote_selftest(uint32_t mask, bool fast);

static void st_led_set(uint8_t idx, bool on);
static void st_led_all_off(void);
static void st_hb_set(bool on);
static void st_eth_reset_set(bool asserted);
static void st_can_pdu_stb_set(bool stb);
static void st_can_left_stb_set(bool stb);
static void st_can_right_stb_set(bool stb);
static void st_esp_reset_set(bool asserted);
static void st_imu0_ncs_set(bool on);
static void st_imu1_ncs_set(bool on);

static uint32_t st_tim1_kernel_clock_hz(void);
static void st_buzzer_off(void);
static void st_buzzer_start(uint32_t hz);
static float st_adc_raw16_to_v(uint16_t raw);
static bool st_therm_read_raw(uint16_t *raw);
static float st_therm_to_c(uint16_t raw);

static bool st_can_loopback_test(FDCAN_HandleTypeDef *hfdcan, const char *tag);
static bool st_eth_phy_read(uint32_t phy_addr, uint32_t reg, uint32_t *value);
static int  st_eth_phy_detect(uint32_t *phy_addr, uint32_t *id1, uint32_t *id2);
static void st_eth_print_basic_status(uint32_t phy_addr);
static bool st_spi_make_8bit(SPI_HandleTypeDef *hspi);
static bool st_imu_read_reg(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ncs_port, uint16_t ncs_pin, uint8_t reg, uint8_t *value);
static bool st_imu_read_burst(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ncs_port, uint16_t ncs_pin, uint8_t start_reg, uint8_t *buf, uint16_t len);
static bool st_imu_config_basic(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ncs_port, uint16_t ncs_pin);
static bool st_imu_read_sample(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ncs_port, uint16_t ncs_pin, st_imu_sample_t *sample);
static SPI_HandleTypeDef *st_imu_spi(uint8_t idx);
static GPIO_TypeDef *st_imu_ncs_port(uint8_t idx);
static uint16_t st_imu_ncs_pin(uint8_t idx);
static const char *st_imu_name(uint8_t idx);
static bool st_imu_prepare(uint8_t idx);
static bool st_imu_read_whoami_idx(uint8_t idx, uint8_t *who);
static bool st_imu_read_sample_idx(uint8_t idx, bool configure, st_imu_sample_t *sample, uint8_t *who);
static void st_imu_print_sample_idx(uint8_t idx, uint8_t who, const st_imu_sample_t *sample);
static st_status_t st_test_imu_common(uint8_t idx, bool expect_present, char *detail, size_t detail_len);
static void st_exp_i2c_scan(void);
static void st_inputs_monitor_print(void);
static void st_therm_monitor_print(void);
static void st_esp_monitor_poll(void);

/* Runtime */
static struct {
    st_runner_mode_t mode;
    uint32_t enabled_mask;
    bool loop_enabled;
    bool loop_fast;
    bool loop_stop_on_fail;
    uint32_t loop_delay_ms;
    uint32_t loop_target_count; /* 0 = infinite */
    uint32_t loop_completed_count;
    uint32_t current_mask;
    int current_test;
    uint32_t wait_until_ms;
    bool monitor_inputs;
    bool monitor_therm;
    bool monitor_esp;
    bool monitor_imu0;
    bool monitor_imu1;
    uint32_t monitor_interval_ms;
    uint32_t last_monitor_ms;
    bool expect_imu1_present;
    bool expect_esp_present;
    bool esp_hold_reset;
    uint8_t cli_buf[ST_CLI_BUF_LEN];
    uint32_t cli_len;
    uint32_t esp_uart_bytes_seen;
    uint32_t imu0_live_last_ms;
    bool campaign_active;
    uint32_t run_fail_count;
    uint32_t run_warn_count;
    float therm_offset_c;
} g_rt = {
    .mode = RUN_IDLE,
    .enabled_mask = ST_TEST_ALL,
    .loop_enabled = false,
    .loop_fast = false,
    .loop_stop_on_fail = false,
    .loop_delay_ms = ST_DEFAULT_LOOP_DELAY_MS,
    .loop_target_count = 0U,
    .loop_completed_count = 0U,
    .current_mask = 0U,
    .current_test = -1,
    .wait_until_ms = 0U,
    .monitor_inputs = false,
    .monitor_therm = false,
    .monitor_esp = false,
    .monitor_imu0 = false,
    .monitor_imu1 = false,
    .monitor_interval_ms = ST_DEFAULT_MONITOR_MS,
    .last_monitor_ms = 0U,
    .expect_imu1_present = false,
    .expect_esp_present = true,
    .esp_hold_reset = false,
    .cli_buf = {0},
    .cli_len = 0U,
    .esp_uart_bytes_seen = 0U,
    .imu0_live_last_ms = 0U,
    .campaign_active = false,
    .run_fail_count = 0U,
    .run_warn_count = 0U,
    .therm_offset_c = 0.0f
};

static st_mcan_rt_t g_mcan;

/* Test contexts */
static struct { uint8_t phase; uint8_t toggles; uint32_t next_ms; } g_gpio_test;
static struct { uint8_t phase; uint32_t next_ms; } g_buzzer_test;
static struct { uint8_t phase; uint32_t next_ms; uint8_t status_before; uint8_t status_after; uint32_t bytes_before; } g_esp_test;
static struct { uint8_t phase; uint32_t deadline_ms; } g_eth_test;
static struct { uint8_t phase; uint32_t deadline_ms; uint32_t base_ping_resp; uint32_t base_ping_timeout; uint32_t base_status_resp; uint32_t base_ack_rx; uint8_t status_seq; uint8_t ack_seq; } g_mcan_test;

/* Forward test funcs */
static void test_gpio_reset(void);    static void test_gpio_start(void);    static st_status_t test_gpio_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_buzzer_reset(void);  static void test_buzzer_start(void);  static st_status_t test_buzzer_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_therm_reset(void);   static void test_therm_start(void);   static st_status_t test_therm_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_pdufault_reset(void);static void test_pdufault_start(void);static st_status_t test_pdufault_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_canpdu_reset(void);  static void test_canpdu_start(void);  static st_status_t test_canpdu_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_canleft_reset(void); static void test_canleft_start(void); static st_status_t test_canleft_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_canright_reset(void);static void test_canright_start(void);static st_status_t test_canright_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_eth_reset(void);     static void test_eth_start(void);     static st_status_t test_eth_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_imu0_reset(void);    static void test_imu0_start(void);    static st_status_t test_imu0_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_imu1_reset(void);    static void test_imu1_start(void);    static st_status_t test_imu1_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_esp_reset(void);     static void test_esp_start(void);     static st_status_t test_esp_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_exp_reset(void);     static void test_exp_start(void);     static st_status_t test_exp_poll(uint32_t now_ms, char *detail, size_t detail_len);
static void test_mcan_reset(void);    static void test_mcan_start(void);    static st_status_t test_mcan_poll(uint32_t now_ms, char *detail, size_t detail_len);

static void test_mcan_reset(void) { memset(&g_mcan_test, 0, sizeof(g_mcan_test)); }
static void test_mcan_start(void)
{
    memset(&g_mcan_test, 0, sizeof(g_mcan_test));
    st_mcan_stop_ping(&g_mcan);
}
static st_status_t test_mcan_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    switch (g_mcan_test.phase) {
        case 0: /* bus warmup — ensure local bus is online */
            if (!g_mcan.online) (void)st_mcan_bus_config(&g_mcan, true);
            if (!g_mcan.online) {
                snprintf(detail, detail_len, "bus config failed");
                return STS_FAIL;
            }
            g_mcan_test.phase = 1;
            return STS_RUNNING;
        case 1:
            g_mcan_test.base_ping_resp = g_mcan.ping_resp;
            g_mcan_test.base_ping_timeout = g_mcan.ping_timeout;
            st_mcan_start_ping(&g_mcan, 1U, 100U, false, false);
            g_mcan_test.deadline_ms = now_ms + 1000U;
            g_mcan_test.phase = 2;
            return STS_RUNNING;
        case 2:
            if (g_mcan.ping_resp > g_mcan_test.base_ping_resp) {
                g_mcan_test.base_status_resp = g_mcan.status_resp_rx;
                g_mcan_test.status_seq = ++g_mcan.next_seq;
                if (!st_mcan_send_simple(&g_mcan, ST_MCAN_TYPE_STATUS_REQ, g_mcan_test.status_seq, 0U, 0U, 0U)) {
                    snprintf(detail, detail_len, "status req send failed");
                    return STS_FAIL;
                }
                g_mcan.tx_req++;
                g_mcan_test.deadline_ms = now_ms + 1000U;
                g_mcan_test.phase = 3;
                return STS_RUNNING;
            }
            if (g_mcan.ping_timeout > g_mcan_test.base_ping_timeout || (int32_t)(now_ms - g_mcan_test.deadline_ms) >= 0) {
                snprintf(detail, detail_len, "ping timeout");
                return STS_FAIL;
            }
            return STS_RUNNING;
        case 3:
            if (g_mcan.status_resp_rx > g_mcan_test.base_status_resp && g_mcan.last_status_seq == g_mcan_test.status_seq) {
                g_mcan_test.base_ack_rx = g_mcan.ack_rx;
                g_mcan_test.ack_seq = ++g_mcan.next_seq;
                if (!st_mcan_send_simple(&g_mcan, ST_MCAN_TYPE_LED_REQ, g_mcan_test.ack_seq, 0U, 1U, 50U)) {
                    snprintf(detail, detail_len, "led req send failed");
                    return STS_FAIL;
                }
                g_mcan.tx_req++;
                g_mcan_test.deadline_ms = now_ms + 1000U;
                g_mcan_test.phase = 4;
                return STS_RUNNING;
            }
            if ((int32_t)(now_ms - g_mcan_test.deadline_ms) >= 0) {
                snprintf(detail, detail_len, "status timeout");
                return STS_FAIL;
            }
            return STS_RUNNING;
        case 4:
            if (g_mcan.ack_rx > g_mcan_test.base_ack_rx &&
                g_mcan.last_ack_seq == g_mcan_test.ack_seq &&
                g_mcan.last_ack_type == ST_MCAN_TYPE_LED_REQ) {
                snprintf(detail, detail_len, "ping/status/ack OK sw=0x%04X", g_mcan.last_status_word);
                return STS_PASS;
            }
            if ((int32_t)(now_ms - g_mcan_test.deadline_ms) >= 0) {
                snprintf(detail, detail_len, "ack timeout");
                return STS_FAIL;
            }
            return STS_RUNNING;
        default:
            return STS_FAIL;
    }
}

static st_test_t g_tests[] = {
    { "gpio",      ST_TEST_GPIO,      test_gpio_reset,    test_gpio_start,    test_gpio_poll,    STS_NOT_RUN, {0} },
    { "buzzer",    ST_TEST_BUZZER,    test_buzzer_reset,  test_buzzer_start,  test_buzzer_poll,  STS_NOT_RUN, {0} },
    { "therm",     ST_TEST_THERM,     test_therm_reset,   test_therm_start,   test_therm_poll,   STS_NOT_RUN, {0} },
    { "pdufault",  ST_TEST_PDUFAULT,  test_pdufault_reset,test_pdufault_start,test_pdufault_poll,STS_NOT_RUN, {0} },
    { "can_pdu",   ST_TEST_CAN_PDU,   test_canpdu_reset,  test_canpdu_start,  test_canpdu_poll,  STS_NOT_RUN, {0} },
    { "can_left",  ST_TEST_CAN_LEFT,  test_canleft_reset, test_canleft_start, test_canleft_poll, STS_NOT_RUN, {0} },
    { "can_right", ST_TEST_CAN_RIGHT, test_canright_reset,test_canright_start,test_canright_poll,STS_NOT_RUN, {0} },
    { "eth",       ST_TEST_ETH,       test_eth_reset,     test_eth_start,     test_eth_poll,     STS_NOT_RUN, {0} },
    { "imu0",      ST_TEST_IMU0,      test_imu0_reset,    test_imu0_start,    test_imu0_poll,    STS_NOT_RUN, {0} },
    { "imu1",      ST_TEST_IMU1,      test_imu1_reset,    test_imu1_start,    test_imu1_poll,    STS_NOT_RUN, {0} },
    { "esp",       ST_TEST_ESP,       test_esp_reset,     test_esp_start,     test_esp_poll,     STS_NOT_RUN, {0} },
    { "exp",       ST_TEST_EXP,       test_exp_reset,     test_exp_start,     test_exp_poll,     STS_NOT_RUN, {0} },
    { "mcan",      ST_TEST_MCAN,      test_mcan_reset,    test_mcan_start,    test_mcan_poll,    STS_NOT_RUN, {0} },
};
#define ST_TEST_COUNT ((int)(sizeof(g_tests)/sizeof(g_tests[0])))

static uint32_t g_campaign_pass[sizeof(g_tests)/sizeof(g_tests[0])];
static uint32_t g_campaign_fail[sizeof(g_tests)/sizeof(g_tests[0])];
static uint32_t g_campaign_warn[sizeof(g_tests)/sizeof(g_tests[0])];
static uint32_t g_campaign_skip[sizeof(g_tests)/sizeof(g_tests[0])];

/* Stream / telemetry mode */
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float temp_c;
    uint32_t last_ok_ms;
    bool valid;
    bool stale;     /* refresh attempted this tick but failed */
} st_imu_stream_cache_t;
static st_imu_stream_cache_t g_imu_stream[2];

static bool     g_stream_on = false;
static uint32_t g_stream_interval_ms = 500U;
static uint32_t g_stream_next_ms = 0U;

static void st_print_campaign_totals(void);

void RCU_SelfTest_Init(void)
{
    st_led_all_off();
    st_buzzer_off();
    st_hb_set(true);
    st_eth_reset_set(false);
    st_can_pdu_stb_set(true);
    st_can_left_stb_set(true);
    st_can_right_stb_set(true);
    st_esp_reset_set(false);
    st_imu0_ncs_set(true);
    st_imu1_ncs_set(true);

    st_reset_all_tests();

    st_mcan_init(&g_mcan);
    if (st_mcan_bus_config(&g_mcan, true) == HAL_OK)
        st_dbg_printf("[MCAN] bus auto-enabled\r\n");

    st_dbg_printf("\r\nRCU SELFTEST READY  %s\r\n", ST_VERSION);
    st_dbg_printf("Type 'help'\r\n");
    st_prompt();
}

void RCU_SelfTest_Task(void)
{
    uint32_t now = HAL_GetTick();
    st_handle_uart_rx();
    st_runner_poll(now);
    st_monitor_poll(now);
    st_stream_poll(now);
    st_mcan_poll(&g_mcan, now);
    st_esp_monitor_poll();
    eth_stack_poll();
}

/* ------------ helpers ------------ */

void st_dbg_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if ((size_t)n > sizeof(buf)) n = (int)sizeof(buf);
    HAL_UART_Transmit(&ST_DBG_UART, (uint8_t *)buf, (uint16_t)n, 100);
}

static void st_prompt(void) { st_dbg_printf("\r\n> "); }

static float st_therm_apply_cal(float t_raw_c)
{
    if (isnan(t_raw_c)) return t_raw_c;
    return t_raw_c + g_rt.therm_offset_c;
}

static void st_therm_print_detail(void)
{
    uint16_t raw = 0U;
    if (!st_therm_read_raw(&raw)) {
        st_dbg_printf("[THERM] read failed\r\n");
        return;
    }

    float v = st_adc_raw16_to_v(raw);
    float t_raw = st_therm_to_c(raw);
    float t = st_therm_apply_cal(t_raw);

    st_dbg_printf("[THERM] raw=%u V=%.4f Traw=%.2fC T=%.2fC offs=%.2fC\r\n",
                  raw, v, t_raw, t, g_rt.therm_offset_c);
}

static void st_led_set(uint8_t idx, bool on)
{
#if ST_LED_ACTIVE_LOW
    GPIO_PinState s = on ? GPIO_PIN_RESET : GPIO_PIN_SET;
#else
    GPIO_PinState s = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif
    if (idx == 0U) HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, s);
    if (idx == 1U) HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, s);
}

static void st_led_all_off(void) { st_led_set(0U, false); st_led_set(1U, false); }
static void st_hb_set(bool on) { HAL_GPIO_WritePin(MCU_HB_GPIO_Port, MCU_HB_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

/* asserted=true means hold reset active */
static void st_eth_reset_set(bool asserted) { HAL_GPIO_WritePin(ETH_NRST_GPIO_Port, ETH_NRST_Pin, asserted ? GPIO_PIN_RESET : GPIO_PIN_SET); }
static void st_can_pdu_stb_set(bool stb)   { HAL_GPIO_WritePin(CAN_PDU_STB_GPIO_Port, CAN_PDU_STB_Pin, stb ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void st_can_left_stb_set(bool stb)  { HAL_GPIO_WritePin(CAN_MTR_L_STB_GPIO_Port, CAN_MTR_L_STB_Pin, stb ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void st_can_right_stb_set(bool stb) { HAL_GPIO_WritePin(CAN_MTR_R_STB_GPIO_Port, CAN_MTR_R_STB_Pin, stb ? GPIO_PIN_SET : GPIO_PIN_RESET); }
/* asserted=true means pull ESP EN low via transistor */
static void st_esp_reset_set(bool asserted) { HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, asserted ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void st_imu0_ncs_set(bool on) { HAL_GPIO_WritePin(IMU0_NCS_GPIO_Port, IMU0_NCS_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void st_imu1_ncs_set(bool on) { HAL_GPIO_WritePin(IMU1_NCS_GPIO_Port, IMU1_NCS_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

static uint32_t st_tim1_kernel_clock_hz(void)
{
    RCC_ClkInitTypeDef clk_init = {0};
    uint32_t flash_latency = 0;
    HAL_RCC_GetClockConfig(&clk_init, &flash_latency);
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    return (clk_init.APB2CLKDivider == RCC_APB2_DIV1) ? pclk2 : (pclk2 * 2U);
}

static void st_buzzer_off(void)
{
    HAL_TIM_PWM_Stop(&ST_BUZZER_TIM, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&ST_BUZZER_TIM, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&ST_BUZZER_TIM, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COUNTER(&ST_BUZZER_TIM, 0U);
}

static void st_buzzer_start(uint32_t hz)
{
    if (hz == 0U) { st_buzzer_off(); return; }
    uint32_t tim_clk = st_tim1_kernel_clock_hz();
    uint32_t psc = ST_BUZZER_TIM.Init.Prescaler + 1U;
    uint32_t period_counts = tim_clk / (psc * hz);
    if (period_counts < 2U) period_counts = 2U;
    if (period_counts > 65536U) period_counts = 65536U;
    uint32_t arr = period_counts - 1U;
    uint32_t ccr = period_counts / 2U;

    HAL_TIM_PWM_Stop(&ST_BUZZER_TIM, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&ST_BUZZER_TIM, TIM_CHANNEL_1);
    __HAL_TIM_SET_AUTORELOAD(&ST_BUZZER_TIM, arr);
    __HAL_TIM_SET_COMPARE(&ST_BUZZER_TIM, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COUNTER(&ST_BUZZER_TIM, 0U);
    HAL_TIM_GenerateEvent(&ST_BUZZER_TIM, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&ST_BUZZER_TIM, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&ST_BUZZER_TIM, TIM_CHANNEL_1);
}

static float st_adc_raw16_to_v(uint16_t raw)
{
    return ((float)raw) * ST_VREF / 65535.0f;
}

static bool st_therm_read_raw(uint16_t *raw)
{
    if (raw == NULL) return false;
    if (HAL_ADC_Start(&ST_THERM_ADC) != HAL_OK) return false;
    if (HAL_ADC_PollForConversion(&ST_THERM_ADC, 20U) != HAL_OK) {
        (void)HAL_ADC_Stop(&ST_THERM_ADC);
        return false;
    }
    uint32_t v = HAL_ADC_GetValue(&ST_THERM_ADC);
    (void)HAL_ADC_Stop(&ST_THERM_ADC);
    *raw = (uint16_t)(v & 0xFFFFU);
    return true;
}

static float st_therm_to_c(uint16_t raw)
{
    float v = st_adc_raw16_to_v(raw);
    if (v <= 0.0f || v >= (ST_VREF - 0.001f)) return NAN;
    float r_ntc = ST_THERM_SERIES_R * (v / (ST_VREF - v));
    float invT = (1.0f / 298.15f) + (1.0f / ST_THERM_BETA) * logf(r_ntc / ST_THERM_R25);
    return (1.0f / invT) - 273.15f;
}

/* Board-provided externs for shared MCAN layer */
FDCAN_HandleTypeDef *st_mcan_get_fdcan(void) {
    return &ST_CAN_PDU;
}

void st_mcan_transceiver_set(bool enable) {
    st_can_pdu_stb_set(!enable);
}

void st_mcan_action_led_set(bool on) {
    st_led_set(0U, on);
    st_led_set(1U, on);
}

void st_mcan_action_buzzer_set(uint32_t freq_hz) {
    if (freq_hz == 0U) st_buzzer_off();
    else st_buzzer_start(freq_hz);
}

static bool st_can_loopback_test(FDCAN_HandleTypeDef *hfdcan, const char *tag)
{
    bool mcan_was_online = (hfdcan == &ST_CAN_PDU) ? g_mcan.online : false;
    if (mcan_was_online) st_mcan_bus_disable(&g_mcan);
    HAL_StatusTypeDef st;
    uint8_t txData[8] = { (uint8_t)tag[0], (uint8_t)tag[1], 0x55, 0xAA, 1U, 2U, 3U, 4U };
    uint8_t rxData[8] = {0};
    FDCAN_FilterTypeDef filter = {0};
    FDCAN_TxHeaderTypeDef txHeader = {0};
    FDCAN_RxHeaderTypeDef rxHeader = {0};
    FDCAN_InitTypeDef init_backup = hfdcan->Init;

    HAL_FDCAN_Stop(hfdcan);
    HAL_FDCAN_DeInit(hfdcan);

    hfdcan->Init = init_backup;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
    hfdcan->Init.AutoRetransmission = DISABLE;
    hfdcan->Init.TransmitPause = DISABLE;
    hfdcan->Init.ProtocolException = DISABLE;

    if (hfdcan->Init.StdFiltersNbr < 1U) hfdcan->Init.StdFiltersNbr = 1U;
    if (hfdcan->Init.RxFifo0ElmtsNbr < 1U) hfdcan->Init.RxFifo0ElmtsNbr = 1U;
    if (hfdcan->Init.TxFifoQueueElmtsNbr < 1U) hfdcan->Init.TxFifoQueueElmtsNbr = 1U;
    hfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    st = HAL_FDCAN_Init(hfdcan);
    if (st != HAL_OK) goto fail_restore;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;
    st = HAL_FDCAN_ConfigFilter(hfdcan, &filter);
    if (st != HAL_OK) goto fail_restore;

    st = HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    if (st != HAL_OK) goto fail_restore;

    st = HAL_FDCAN_Start(hfdcan);
    if (st != HAL_OK) goto fail_restore;

    txHeader.Identifier = 0x123U;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0U;

    st = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData);
    if (st != HAL_OK) goto fail_restore;

    uint32_t t0 = HAL_GetTick();
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0U) {
        if ((HAL_GetTick() - t0) > 100U) goto fail_restore;
    }

    st = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData);
    if (st != HAL_OK) goto fail_restore;

    HAL_FDCAN_Stop(hfdcan);
    HAL_FDCAN_DeInit(hfdcan);
    hfdcan->Init = init_backup;
    (void)HAL_FDCAN_Init(hfdcan);
    if (mcan_was_online) (void)st_mcan_bus_config(&g_mcan, true);
    return (rxHeader.Identifier == 0x123U && memcmp(txData, rxData, 8) == 0);

fail_restore:
    HAL_FDCAN_Stop(hfdcan);
    HAL_FDCAN_DeInit(hfdcan);
    hfdcan->Init = init_backup;
    (void)HAL_FDCAN_Init(hfdcan);
    if (mcan_was_online) (void)st_mcan_bus_config(&g_mcan, true);
    return false;
}

static bool st_eth_phy_read(uint32_t phy_addr, uint32_t reg, uint32_t *value)
{
    return HAL_ETH_ReadPHYRegister(&ST_ETH, phy_addr, reg, value) == HAL_OK;
}

static int st_eth_phy_detect(uint32_t *phy_addr, uint32_t *id1, uint32_t *id2)
{
    uint32_t v1 = 0, v2 = 0;
    for (uint32_t addr = 0; addr < 32U; ++addr) {
        if (!st_eth_phy_read(addr, ST_ETH_PHY_ID1, &v1)) continue;
        if (!st_eth_phy_read(addr, ST_ETH_PHY_ID2, &v2)) continue;
        if (v1 != 0x0000U && v1 != 0xFFFFU && v2 != 0x0000U && v2 != 0xFFFFU) {
            if (phy_addr) *phy_addr = addr;
            if (id1) *id1 = v1;
            if (id2) *id2 = v2;
            return 1;
        }
    }
    return 0;
}

static void st_eth_print_basic_status(uint32_t phy_addr)
{
    uint32_t bmsr = 0, bmcr = 0, id1 = 0, id2 = 0;
    if (!st_eth_phy_read(phy_addr, ST_ETH_PHY_BMCR, &bmcr) ||
        !st_eth_phy_read(phy_addr, ST_ETH_PHY_BMSR, &bmsr) ||
        !st_eth_phy_read(phy_addr, ST_ETH_PHY_ID1, &id1) ||
        !st_eth_phy_read(phy_addr, ST_ETH_PHY_ID2, &id2)) {
        st_dbg_printf("[ETH] PHY read failed\r\n");
        return;
    }
    st_dbg_printf("[ETH] PHY@%lu ID1=0x%04lX ID2=0x%04lX BMCR=0x%04lX BMSR=0x%04lX LINK=%u ANEG=%u\r\n",
                  (unsigned long)phy_addr,
                  (unsigned long)(id1 & 0xFFFFU), (unsigned long)(id2 & 0xFFFFU),
                  (unsigned long)(bmcr & 0xFFFFU), (unsigned long)(bmsr & 0xFFFFU),
                  (bmsr & ST_ETH_BMSR_LINK) ? 1U : 0U,
                  (bmsr & ST_ETH_BMSR_AUTONEGCOMP) ? 1U : 0U);
}

static bool st_spi_make_8bit(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_DeInit(hspi);
    hspi->Init.DataSize = SPI_DATASIZE_8BIT;
    hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hspi->Init.Mode = SPI_MODE_MASTER;
    hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi->Init.NSS = SPI_NSS_SOFT;
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* conservative */
    hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi->Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi->Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi->Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi->Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi->Init.IOSwap = SPI_IO_SWAP_DISABLE;
    return HAL_SPI_Init(hspi) == HAL_OK;
}

static bool st_imu_read_reg(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ncs_port, uint16_t ncs_pin, uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80U), 0x00U };
    uint8_t rx[2] = { 0, 0 };
    HAL_GPIO_WritePin(ncs_port, ncs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, 2U, 50U);
    HAL_GPIO_WritePin(ncs_port, ncs_pin, GPIO_PIN_SET);
    if (st != HAL_OK) return false;
    *value = rx[1];
    return true;
}

static bool st_imu_write_reg(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ncs_port, uint16_t ncs_pin, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), value };
    HAL_GPIO_WritePin(ncs_port, ncs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, tx, 2U, 50U);
    HAL_GPIO_WritePin(ncs_port, ncs_pin, GPIO_PIN_SET);
    return st == HAL_OK;
}

static bool st_imu_read_burst(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ncs_port, uint16_t ncs_pin, uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    if (len == 0U) return false;
    uint8_t hdr = (uint8_t)(start_reg | 0x80U);
    HAL_GPIO_WritePin(ncs_port, ncs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, &hdr, 1U, 50U);
    if (st == HAL_OK) st = HAL_SPI_Receive(hspi, buf, len, 100U);
    HAL_GPIO_WritePin(ncs_port, ncs_pin, GPIO_PIN_SET);
    return st == HAL_OK;
}

static bool st_imu_config_basic(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ncs_port, uint16_t ncs_pin)
{
    /* IF_INC + BDU */
    if (!st_imu_write_reg(hspi, ncs_port, ncs_pin, ST_IMU_CTRL3_C, 0x44U)) return false;
    /* 104 Hz accel 2g, 104 Hz gyro 250 dps */
    if (!st_imu_write_reg(hspi, ncs_port, ncs_pin, ST_IMU_CTRL1_XL, 0x40U)) return false;
    if (!st_imu_write_reg(hspi, ncs_port, ncs_pin, ST_IMU_CTRL2_G, 0x40U)) return false;
    return true;
}

static bool st_imu_read_sample(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ncs_port, uint16_t ncs_pin, st_imu_sample_t *sample)
{
    uint8_t buf[14];
    if (!st_imu_read_burst(hspi, ncs_port, ncs_pin, ST_IMU_OUT_TEMP_L, buf, sizeof(buf))) return false;
    sample->temp_raw = (int16_t)((buf[1] << 8) | buf[0]);
    sample->gyro[0]  = (int16_t)((buf[3] << 8) | buf[2]);
    sample->gyro[1]  = (int16_t)((buf[5] << 8) | buf[4]);
    sample->gyro[2]  = (int16_t)((buf[7] << 8) | buf[6]);
    sample->accel[0] = (int16_t)((buf[9] << 8) | buf[8]);
    sample->accel[1] = (int16_t)((buf[11] << 8) | buf[10]);
    sample->accel[2] = (int16_t)((buf[13] << 8) | buf[12]);
    return true;
}

static SPI_HandleTypeDef *st_imu_spi(uint8_t idx)
{
    return (idx == 0U) ? &ST_IMU0_SPI : &ST_IMU1_SPI;
}

static GPIO_TypeDef *st_imu_ncs_port(uint8_t idx)
{
    return (idx == 0U) ? IMU0_NCS_GPIO_Port : IMU1_NCS_GPIO_Port;
}

static uint16_t st_imu_ncs_pin(uint8_t idx)
{
    return (idx == 0U) ? IMU0_NCS_Pin : IMU1_NCS_Pin;
}

static const char *st_imu_name(uint8_t idx)
{
    return (idx == 0U) ? "IMU0" : "IMU1";
}

static bool st_imu_prepare(uint8_t idx)
{
    SPI_HandleTypeDef *spi = st_imu_spi(idx);
    GPIO_TypeDef *port = st_imu_ncs_port(idx);
    uint16_t pin = st_imu_ncs_pin(idx);
    if (!st_spi_make_8bit(spi)) return false;
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    return true;
}

static bool st_imu_read_whoami_idx(uint8_t idx, uint8_t *who)
{
    return st_imu_read_reg(st_imu_spi(idx), st_imu_ncs_port(idx), st_imu_ncs_pin(idx), ST_IMU_WHOAMI_REG, who);
}

static bool st_imu_read_sample_idx(uint8_t idx, bool configure, st_imu_sample_t *sample, uint8_t *who)
{
    if (!st_imu_prepare(idx)) return false;
    if (who != NULL) {
        if (!st_imu_read_whoami_idx(idx, who)) return false;
    }
    if (configure) {
        if (!st_imu_config_basic(st_imu_spi(idx), st_imu_ncs_port(idx), st_imu_ncs_pin(idx))) return false;
        HAL_Delay(20U);
    }
    return st_imu_read_sample(st_imu_spi(idx), st_imu_ncs_port(idx), st_imu_ncs_pin(idx), sample);
}

static void st_imu_print_sample_idx(uint8_t idx, uint8_t who, const st_imu_sample_t *sample)
{
    st_dbg_printf("[%s] WHO=0x%02X Traw=%d G=(%d,%d,%d) A=(%d,%d,%d)\r\n",
                  st_imu_name(idx), who,
                  sample->temp_raw,
                  sample->gyro[0], sample->gyro[1], sample->gyro[2],
                  sample->accel[0], sample->accel[1], sample->accel[2]);
}

static st_status_t st_test_imu_common(uint8_t idx, bool expect_present, char *detail, size_t detail_len)
{
    uint8_t who = 0U;
    st_imu_sample_t s;

    if (!st_imu_prepare(idx)) {
        snprintf(detail, detail_len, "SPI%d 8-bit init failed", idx == 0U ? 4 : 3);
        return STS_FAIL;
    }

    if (!st_imu_read_whoami_idx(idx, &who)) {
        snprintf(detail, detail_len, "WHO_AM_I read failed");
        return expect_present ? STS_FAIL : STS_SKIPPED;
    }

    if (who == 0x00U || who == 0xFFU) {
        snprintf(detail, detail_len, "WHO_AM_I=0x%02X", who);
        return expect_present ? STS_FAIL : STS_SKIPPED;
    }

    if (who != ST_LSM6DSOX_WHOAMI) {
        snprintf(detail, detail_len, "WHO_AM_I=0x%02X expected 0x%02X", who, ST_LSM6DSOX_WHOAMI);
        return STS_WARN;
    }

    if (!st_imu_read_sample_idx(idx, true, &s, NULL)) {
        snprintf(detail, detail_len, "WHO=0x%02X sample read failed", who);
        return expect_present ? STS_WARN : STS_SKIPPED;
    }

    snprintf(detail, detail_len, "WHO=0x%02X Traw=%d G=(%d,%d,%d) A=(%d,%d,%d)",
             who, s.temp_raw, s.gyro[0], s.gyro[1], s.gyro[2], s.accel[0], s.accel[1], s.accel[2]);
    return STS_PASS;
}

static void st_exp_i2c_scan(void)
{
    st_dbg_printf("[EXP_I2C] scan\r\n");
    for (uint8_t a = 1U; a < 0x7FU; ++a) {
        if (HAL_I2C_IsDeviceReady(&ST_EXP_I2C, (uint16_t)(a << 1), 1U, 5U) == HAL_OK) {
            st_dbg_printf("  0x%02X\r\n", a);
        }
    }
}

static void st_inputs_monitor_print(void)
{
    st_dbg_printf("[IN] PDU_FAULT=%u ESP_STATUS=%u IMU0_INT1=%u IMU0_INT2=%u IMU1_INT1=%u EXP=%u%u%u%u\r\n",
                  HAL_GPIO_ReadPin(PDU_FAULT_GPIO_Port, PDU_FAULT_Pin),
                  HAL_GPIO_ReadPin(ESP_STATUS_GPIO_Port, ESP_STATUS_Pin),
                  HAL_GPIO_ReadPin(IMU0_INT1_GPIO_Port, IMU0_INT1_Pin),
                  HAL_GPIO_ReadPin(IMU0_INT2_GPIO_Port, IMU0_INT2_Pin),
                  HAL_GPIO_ReadPin(IMU1_INT1_GPIO_Port, IMU1_INT1_Pin),
                  HAL_GPIO_ReadPin(EXP_GPIO0_GPIO_Port, EXP_GPIO0_Pin),
                  HAL_GPIO_ReadPin(EXP_GPIO1_GPIO_Port, EXP_GPIO1_Pin),
                  HAL_GPIO_ReadPin(EXP_GPIO2_GPIO_Port, EXP_GPIO2_Pin),
                  HAL_GPIO_ReadPin(EXP_GPIO3_GPIO_Port, EXP_GPIO3_Pin));
}

static void st_therm_monitor_print(void)
{
    uint16_t raw = 0;
    if (!st_therm_read_raw(&raw)) {
        st_dbg_printf("[THERM] read failed\r\n");
        return;
    }
    float v = st_adc_raw16_to_v(raw);
    float t = st_therm_to_c(raw);
    st_dbg_printf("[THERM] raw=%u V=%.4f T=%.2fC\r\n", raw, v, t);
}

static void st_esp_monitor_poll(void)
{
    uint8_t ch;
    while (HAL_UART_Receive(&ST_ESP_UART, &ch, 1U, 0U) == HAL_OK) {
        g_rt.esp_uart_bytes_seen++;
        if (g_rt.monitor_esp) {
            if (ch == '\r') st_dbg_printf("<CR>");
            else if (ch == '\n') st_dbg_printf("<LF>\r\n");
            else if (isprint((unsigned char)ch)) st_dbg_printf("%c", ch);
            else st_dbg_printf("<%02X>", ch);
        }
    }
}

/* -------- CLI / runner -------- */

static void st_reset_all_tests(void)
{
    for (int i = 0; i < ST_TEST_COUNT; ++i) {
        g_tests[i].status = STS_NOT_RUN;
        g_tests[i].detail[0] = '\0';
        if (g_tests[i].reset) g_tests[i].reset();
    }
}

static int st_test_index_from_name(const char *name)
{
    for (int i = 0; i < ST_TEST_COUNT; ++i) {
        if (st_str_eq_nocase(name, g_tests[i].name)) return i;
    }
    return -1;
}

static void st_start_one(uint32_t mask)
{
    st_reset_all_tests();
    g_rt.mode = RUN_ONE;
    g_rt.current_mask = mask;
    g_rt.current_test = -1;
    g_rt.run_fail_count = 0U;
    g_rt.run_warn_count = 0U;

    for (int i = 0; i < ST_TEST_COUNT; ++i) {
        if (g_tests[i].mask & mask) {
            g_rt.current_test = i;
            g_tests[i].status = STS_RUNNING;
            g_tests[i].detail[0] = '\0';
            if (g_tests[i].start) g_tests[i].start();
            st_dbg_printf("[RUN] %s\r\n", g_tests[i].name);
            return;
        }
    }

    g_rt.mode = RUN_IDLE;
}

static void st_start_all(void)
{
    st_reset_all_tests();
    g_rt.mode = RUN_ALL;
    g_rt.current_test = -1;
    g_rt.current_mask = g_rt.enabled_mask;
    g_rt.run_fail_count = 0U;
    g_rt.run_warn_count = 0U;

    for (int i = 0; i < ST_TEST_COUNT; ++i) {
        if (g_tests[i].mask & g_rt.enabled_mask) {
            g_rt.current_test = i;
            g_tests[i].status = STS_RUNNING;
            if (g_tests[i].start) g_tests[i].start();
            if (!g_rt.loop_fast) st_dbg_printf("[RUN] %s\r\n", g_tests[i].name);
            return;
        } else {
            g_tests[i].status = STS_SKIPPED;
        }
    }

    g_rt.mode = RUN_IDLE;
}

/* Board-provided externs for shared MCAN layer (Group B) */
uint8_t st_mcan_get_summary_code(void) {
    bool any_fail = false, any_warn = false;
    for (int i = 0; i < ST_TEST_COUNT; ++i) {
        if (g_tests[i].status == STS_FAIL) any_fail = true;
        else if (g_tests[i].status == STS_WARN) any_warn = true;
    }
    return any_fail ? 2U : (any_warn ? 1U : 0U);
}

uint16_t st_mcan_get_status_word(void) {
    uint16_t w = 0U;
    if (HAL_GPIO_ReadPin(PDU_FAULT_GPIO_Port, PDU_FAULT_Pin) == GPIO_PIN_SET) w |= (1U << 0);
    if (HAL_GPIO_ReadPin(ESP_STATUS_GPIO_Port, ESP_STATUS_Pin) == GPIO_PIN_SET) w |= (1U << 1);
    return w;
}

uint32_t st_mcan_local_mask_from_selector(uint16_t selector) {
    switch (selector) {
        case ST_MCAN_SEL_ALL:       return ST_TEST_ALL;
        case ST_MCAN_SEL_GPIO:      return ST_TEST_GPIO;
        case ST_MCAN_SEL_BUZZER:    return ST_TEST_BUZZER;
        case ST_MCAN_SEL_MCAN:      return ST_TEST_MCAN;
        case ST_MCAN_SEL_THERM:     return ST_TEST_THERM;
        case ST_MCAN_SEL_PDUFAULT:  return ST_TEST_PDUFAULT;
        case ST_MCAN_SEL_CAN_PDU:   return ST_TEST_CAN_PDU;
        case ST_MCAN_SEL_CAN_LEFT:  return ST_TEST_CAN_LEFT;
        case ST_MCAN_SEL_CAN_RIGHT: return ST_TEST_CAN_RIGHT;
        case ST_MCAN_SEL_ETH:       return ST_TEST_ETH;
        case ST_MCAN_SEL_IMU0:      return ST_TEST_IMU0;
        case ST_MCAN_SEL_IMU1:      return ST_TEST_IMU1;
        case ST_MCAN_SEL_ESP:       return ST_TEST_ESP;
        case ST_MCAN_SEL_EXP:       return ST_TEST_EXP;
        default: return 0U;
    }
}

void st_mcan_on_start_remote_selftest(uint32_t mask, bool fast) {
    g_rt.loop_fast = fast;
    g_rt.campaign_active = false;
    if (mask == ST_TEST_ALL) st_start_all();
    else st_start_one(mask);
}

static void st_abort(bool announce)
{
    st_led_all_off();
    st_buzzer_off();
    g_rt.mode = RUN_IDLE;
    g_rt.current_test = -1;
    g_rt.current_mask = 0U;
    g_rt.campaign_active = false;
    g_rt.loop_enabled = false;
    g_rt.loop_target_count = 0U;

    if (announce) st_dbg_printf("[RUN] stopped\r\n");
}

static void st_runner_poll(uint32_t now_ms)
{
    if (g_rt.mode == RUN_IDLE) return;

    if (g_rt.mode == RUN_WAIT_LOOP) {
        if ((int32_t)(now_ms - g_rt.wait_until_ms) >= 0) {
            if (g_rt.campaign_active) {
                if (g_rt.loop_target_count != 0U &&
                    g_rt.loop_completed_count >= g_rt.loop_target_count) {
                    g_rt.mode = RUN_IDLE;
                    st_dbg_printf("[LOOP] done\r\n");
                    st_print_campaign_totals();
                    st_prompt();
                    g_rt.campaign_active = false;
                    g_rt.loop_enabled = false;
                    return;
                }
                st_start_all();
            } else {
                g_rt.mode = RUN_IDLE;
                st_prompt();
            }
        }
        return;
    }

    if (g_rt.current_test < 0 || g_rt.current_test >= ST_TEST_COUNT) {
        g_rt.mode = RUN_IDLE;
        return;
    }

    st_test_t *t = &g_tests[g_rt.current_test];
    st_status_t s = t->poll ? t->poll(now_ms, t->detail, sizeof(t->detail)) : STS_FAIL;
    if (s == STS_RUNNING) return;

    t->status = s;
    if (s == STS_FAIL) g_rt.run_fail_count++;
    else if (s == STS_WARN) g_rt.run_warn_count++;
    if (g_rt.campaign_active) {
        if (s == STS_PASS) g_campaign_pass[g_rt.current_test]++;
        else if (s == STS_FAIL) g_campaign_fail[g_rt.current_test]++;
        else if (s == STS_WARN) g_campaign_warn[g_rt.current_test]++;
        else if (s == STS_SKIPPED) g_campaign_skip[g_rt.current_test]++;
    }
    if (g_mcan.remote_selftest_active) {
        int sel = st_mcan_selector_from_name(t->name);
        if (sel >= 0)
            (void)st_mcan_send_selftest_result(&g_mcan, g_mcan.remote_selftest_req_seq, (uint16_t)sel, s);
    }

    if (!g_rt.loop_fast) {
        st_dbg_printf("[DONE] %-10s %s", t->name, st_status_str(s));
        if (t->detail[0] != '\0') st_dbg_printf("  %s", t->detail);
        st_dbg_printf("\r\n");
    }

    if (g_rt.campaign_active && g_rt.loop_stop_on_fail && s == STS_FAIL) {
        g_rt.loop_completed_count++;
        st_dbg_printf("[LOOP] stop-on-fail at pass %lu test %s\r\n",
                      (unsigned long)g_rt.loop_completed_count, t->name);
        st_print_summary();
        st_print_campaign_totals();
        if (g_mcan.remote_selftest_active) {
            (void)st_mcan_send_selftest_done(&g_mcan, g_mcan.remote_selftest_req_seq, (uint16_t)g_rt.run_fail_count, (uint16_t)g_rt.run_warn_count);
            g_mcan.remote_selftest_active = false;
        }
        g_rt.mode = RUN_IDLE;
        g_rt.campaign_active = false;
        g_rt.loop_enabled = false;
        st_prompt();
        return;
    }

    int next = -1;
    for (int i = g_rt.current_test + 1; i < ST_TEST_COUNT; ++i) {
        if (g_rt.current_mask & g_tests[i].mask) {
            next = i;
            break;
        } else if (g_rt.mode == RUN_ALL) {
            g_tests[i].status = STS_SKIPPED;
        }
    }

    if (next >= 0) {
        g_rt.current_test = next;
        g_tests[next].status = STS_RUNNING;
        g_tests[next].detail[0] = '\0';
        if (g_tests[next].start) g_tests[next].start();
        if (!g_rt.loop_fast) st_dbg_printf("[RUN] %s\r\n", g_tests[next].name);
    } else {
        st_print_summary();

        if (g_rt.campaign_active) {
            g_rt.loop_completed_count++;
            st_dbg_printf("[LOOP] pass %lu", (unsigned long)g_rt.loop_completed_count);
            if (g_rt.run_fail_count == 0U && g_rt.run_warn_count == 0U) st_dbg_printf(" PASS");
            else st_dbg_printf(" fail=%lu warn=%lu",
                               (unsigned long)g_rt.run_fail_count,
                               (unsigned long)g_rt.run_warn_count);
            st_dbg_printf("\r\n");

            if (g_rt.loop_target_count != 0U &&
                g_rt.loop_completed_count >= g_rt.loop_target_count) {
                if (g_mcan.remote_selftest_active) {
                    (void)st_mcan_send_selftest_done(&g_mcan, g_mcan.remote_selftest_req_seq, (uint16_t)g_rt.run_fail_count, (uint16_t)g_rt.run_warn_count);
                    g_mcan.remote_selftest_active = false;
                }
                g_rt.mode = RUN_IDLE;
                g_rt.campaign_active = false;
                g_rt.loop_enabled = false;
                st_print_campaign_totals();
                st_prompt();
            } else {
                g_rt.mode = RUN_WAIT_LOOP;
                g_rt.wait_until_ms = now_ms + g_rt.loop_delay_ms;
                if (!g_rt.loop_fast) {
                    st_dbg_printf("[LOOP] waiting %lu ms\r\n",
                                  (unsigned long)g_rt.loop_delay_ms);
                }
            }
        } else if (g_rt.loop_enabled) {
            g_rt.mode = RUN_WAIT_LOOP;
            g_rt.wait_until_ms = now_ms + g_rt.loop_delay_ms;
            if (!g_rt.loop_fast) {
                st_dbg_printf("[LOOP] waiting %lu ms\r\n",
                              (unsigned long)g_rt.loop_delay_ms);
            }
        } else {
            if (g_mcan.remote_selftest_active) {
                (void)st_mcan_send_selftest_done(&g_mcan, g_mcan.remote_selftest_req_seq, (uint16_t)g_rt.run_fail_count, (uint16_t)g_rt.run_warn_count);
                g_mcan.remote_selftest_active = false;
            }
            g_rt.mode = RUN_IDLE;
            st_prompt();
        }
    }
}

static void st_monitor_poll(uint32_t now_ms)
{
    if (!g_rt.monitor_inputs && !g_rt.monitor_therm && !g_rt.monitor_imu0 && !g_rt.monitor_imu1) return;
    if ((now_ms - g_rt.last_monitor_ms) < g_rt.monitor_interval_ms) return;
    g_rt.last_monitor_ms = now_ms;

    if (g_rt.monitor_inputs) st_inputs_monitor_print();
    if (g_rt.monitor_therm) st_therm_monitor_print();

    if (g_rt.monitor_imu0 || g_rt.monitor_imu1) {
        for (uint8_t idx = 0U; idx < 2U; ++idx) {
            if ((idx == 0U && !g_rt.monitor_imu0) || (idx == 1U && !g_rt.monitor_imu1)) continue;
            uint8_t who = 0U;
            st_imu_sample_t sample;
            if (st_imu_read_sample_idx(idx, true, &sample, &who)) {
                st_imu_print_sample_idx(idx, who, &sample);
            } else {
                st_dbg_printf("[%s] read failed\r\n", st_imu_name(idx));
            }
        }
    }
}


/* Non-blocking IMU sample into stream cache.
 * Configures the IMU on the first call (valid==false) to set ODR/range, then
 * reads without reconfiguring on subsequent calls.
 * On success: updates g_imu_stream[idx], clears stale flag.
 * On failure: sets stale flag, leaves prior engineering values intact. */
static bool st_imu_cache_refresh(uint8_t idx)
{
    /* Configure on first use; skip thereafter to avoid 20ms HAL_Delay per tick */
    bool need_configure = !g_imu_stream[idx].valid;
    st_imu_sample_t s;
    if (!st_imu_read_sample_idx(idx, need_configure, &s, NULL)) {
        g_imu_stream[idx].stale = true;
        return false;
    }
    /* ±2g at 104 Hz, 0.061 mg/LSB = 0.000061 g/LSB */
    g_imu_stream[idx].ax = (float)s.accel[0] * 0.000061f;
    g_imu_stream[idx].ay = (float)s.accel[1] * 0.000061f;
    g_imu_stream[idx].az = (float)s.accel[2] * 0.000061f;
    /* ±250 dps, 8.75 mdps/LSB */
    g_imu_stream[idx].gx = (float)s.gyro[0] * 0.00875f;
    g_imu_stream[idx].gy = (float)s.gyro[1] * 0.00875f;
    g_imu_stream[idx].gz = (float)s.gyro[2] * 0.00875f;
    g_imu_stream[idx].temp_c = (float)s.temp_raw / 256.0f + 25.0f;
    g_imu_stream[idx].last_ok_ms = HAL_GetTick();
    g_imu_stream[idx].valid = true;
    g_imu_stream[idx].stale = false;
    return true;
}

static void st_stream_poll(uint32_t now_ms)
{
    if (!g_stream_on) return;
    if ((int32_t)(now_ms - g_stream_next_ms) < 0) return;
    g_stream_next_ms = now_ms + g_stream_interval_ms;

    /* --- IMU0 (always) ---------------------------------------------------- */
    bool imu0_ok = st_imu_cache_refresh(0U);

    /* --- IMU1 (optional) -------------------------------------------------- */
    bool imu1_present = g_rt.expect_imu1_present;
    bool imu1_ok = false;
    if (imu1_present) imu1_ok = st_imu_cache_refresh(1U);

    /* --- Temperature ------------------------------------------------------ */
    uint16_t therm_raw = 0U;
    bool therm_ok = st_therm_read_raw(&therm_raw);
    float temp_c = therm_ok ? st_therm_apply_cal(st_therm_to_c(therm_raw)) : 0.0f;

    /* --- PDU status word from last MCAN heartbeat / status resp ----------- */
    uint16_t pdu_sw = g_mcan.last_status_word;
    uint8_t  fpga_state_idx = (uint8_t)((pdu_sw & MCAN_PDU_SW_FPGA_STATE_MASK) >> MCAN_PDU_SW_FPGA_STATE_SHIFT);
    static const char * const k_fpga_state[] = {"IDLE","PRECHARGE","ARMED","COMPUTE"};

    /* --- MCAN RTT --------------------------------------------------------- */
    const char *hb_str = g_mcan.peer_online ? "OK" : "OFF";

    /* --- ETH -------------------------------------------------------------- */
    bool eth_up = eth_stack_is_up();

    /* --- Compose line ----------------------------------------------------- */

    /* IMU0 segment */
    if (!imu0_ok && !g_imu_stream[0].valid) {
        st_dbg_printf("[STR] t=%lu IMU0=NA", (unsigned long)now_ms);
    } else {
        const char *mark0 = g_imu_stream[0].stale ? "!" : "";
        st_dbg_printf("[STR] t=%lu IMU0 ax=%s%.2f ay=%s%.2f az=%s%.2f",
                      (unsigned long)now_ms,
                      mark0, g_imu_stream[0].ax,
                      mark0, g_imu_stream[0].ay,
                      mark0, g_imu_stream[0].az);
    }

    /* IMU1 segment */
    if (imu1_present) {
        if (!imu1_ok && !g_imu_stream[1].valid) {
            st_dbg_printf(" | IMU1=NA");
        } else {
            const char *mark1 = g_imu_stream[1].stale ? "!" : "";
            st_dbg_printf(" | IMU1 ax=%s%.2f ay=%s%.2f az=%s%.2f",
                          mark1, g_imu_stream[1].ax,
                          mark1, g_imu_stream[1].ay,
                          mark1, g_imu_stream[1].az);
        }
    }

    /* Temperature */
    if (therm_ok) st_dbg_printf(" | T=%.1fC", temp_c);
    else          st_dbg_printf(" | T=NA");

    /* PDU status (decoded from MCAN status word) */
    if (g_mcan.peer_online) {
        st_dbg_printf(" | PDU fpga=%s arm=%u estop=%u pgs=%u",
                      k_fpga_state[fpga_state_idx & 0x3U],
                      (pdu_sw & MCAN_PDU_SW_ARM_PERMIT) ? 1U : 0U,
                      (pdu_sw & MCAN_PDU_SW_ESTOP_OK)   ? 1U : 0U,
                      (pdu_sw & MCAN_PDU_SW_PGOOD_SW)   ? 1U : 0U);
    } else {
        st_dbg_printf(" | PDU=offline");
    }

    /* MCAN stats */
    if (g_mcan.peer_selftest_valid) {
        st_dbg_printf(" | MCAN hb=%s rtt=%lums f=%u w=%u",
                      hb_str, (unsigned long)g_mcan.rtt_last_ms,
                      g_mcan.peer_selftest_fail, g_mcan.peer_selftest_warn);
    } else {
        st_dbg_printf(" | MCAN hb=%s rtt=%lums f=? w=?",
                      hb_str, (unsigned long)g_mcan.rtt_last_ms);
    }

    /* ETH */
    if (eth_up) {
        st_dbg_printf(" | ETH up:192.168.100.10");
    } else {
        st_dbg_printf(" | ETH down");
    }

    st_dbg_printf("\r\n");
}

static void st_print_help(void)
{
    st_dbg_printf("=== RCU Self-Test CLI ===\r\n");
    st_dbg_printf("[TESTS]   run all|<test> [fast] [both]\r\n");
    st_dbg_printf("          enable|disable <test>|all\r\n");
    st_dbg_printf("          list | status | summary | clear | stop\r\n");
    st_dbg_printf("[LOOP]    loop on|off | loop count <n> [fast] | loop delay <ms>\r\n");
    st_dbg_printf("          loop fast on|off | loop stoponfail on|off\r\n");
    st_dbg_printf("[STREAM]  stream on|off [ms]\r\n");
    st_dbg_printf("[MONITOR] monitor inputs|therm|esp|imu0|imu1 on|off\r\n");
    st_dbg_printf("          monitor interval <ms>\r\n");
    st_dbg_printf("[IMU]     imu0|imu1 whoami|read|live on|off\r\n");
    st_dbg_printf("          expect imu1 absent|present\r\n");
    st_dbg_printf("[THERM]   therm raw | therm cal show|clear|offset <degC>\r\n");
    st_dbg_printf("[CAN]     can pdu|left|right loopback\r\n");
    st_dbg_printf("          can pdu|left|right stb <0|1>\r\n");
    st_dbg_printf("[MCAN]    mcan monitor|diag|hb|ping|pingboth|status|led|beep\r\n");
    st_dbg_printf("          mcan selftest|stats|clear|enable|reset\r\n");
    st_dbg_printf("[ETH]     eth reset|id|regs|link|up|down|ip|stats|udp on|off\r\n");
    st_dbg_printf("[ESP]     esp reset|status | esp holdreset on|off\r\n");
    st_dbg_printf("[EXP]     exp gpio | exp i2c scan\r\n");
    st_dbg_printf("[GPIO]    set led0|led1|hb|eth_nrst|can_pdu_stb|can_left_stb|can_right_stb|esp_rst <0|1>\r\n");
    st_dbg_printf("[BUZZER]  beep <hz> <ms>\r\n");
}

static void st_print_tests(void)
{
    for (int i = 0; i < ST_TEST_COUNT; ++i) {
        st_dbg_printf("  %-10s enabled=%u status=%s\r\n", g_tests[i].name,
                      (g_rt.enabled_mask & g_tests[i].mask) ? 1U : 0U,
                      st_status_str(g_tests[i].status));
    }
}

static void st_print_status(void)
{
    st_dbg_printf("[RT] mode=%d enabled=0x%08lX loop=%u fast=%u stoponfail=%u target=%lu done=%lu delay=%lu mon=%lu imu0=%u imu1=%u esp_hold=%u imu1_expect=%u therm_offs=%.2f\r\n",
                  (int)g_rt.mode,
                  (unsigned long)g_rt.enabled_mask,
                  g_rt.loop_enabled ? 1U : 0U,
                  g_rt.loop_fast ? 1U : 0U,
                  g_rt.loop_stop_on_fail ? 1U : 0U,
                  (unsigned long)g_rt.loop_target_count,
                  (unsigned long)g_rt.loop_completed_count,
                  (unsigned long)g_rt.loop_delay_ms,
                  (unsigned long)g_rt.monitor_interval_ms,
                  g_rt.monitor_imu0 ? 1U : 0U,
                  g_rt.monitor_imu1 ? 1U : 0U,
                  g_rt.esp_hold_reset ? 1U : 0U,
                  g_rt.expect_imu1_present ? 1U : 0U,
                  g_rt.therm_offset_c);
}

static void st_print_summary(void)
{
    uint32_t pass = 0U, fail = 0U, warn = 0U, skip = 0U;

    for (int i = 0; i < ST_TEST_COUNT; ++i) {
        switch (g_tests[i].status) {
            case STS_PASS:    pass++; break;
            case STS_FAIL:    fail++; break;
            case STS_WARN:    warn++; break;
            case STS_SKIPPED: skip++; break;
            default: break;
        }
    }

    st_dbg_printf("[SUMMARY] pass=%lu fail=%lu warn=%lu skip=%lu\r\n",
                  (unsigned long)pass,
                  (unsigned long)fail,
                  (unsigned long)warn,
                  (unsigned long)skip);

    for (int i = 0; i < ST_TEST_COUNT; ++i) {
        st_dbg_printf("  %-10s %s", g_tests[i].name, st_status_str(g_tests[i].status));
        if (g_tests[i].detail[0] != '\0') st_dbg_printf("  %s", g_tests[i].detail);
        st_dbg_printf("\r\n");
    }
}

static void st_print_campaign_totals(void) {
    st_dbg_printf("[CAMPAIGN TOTALS] passes=%lu\r\n",
                  (unsigned long)g_rt.loop_completed_count);
    for (int i = 0; i < ST_TEST_COUNT; ++i) {
        st_dbg_printf("  %-10s P=%lu F=%lu W=%lu S=%lu\r\n",
                      g_tests[i].name, (unsigned long)g_campaign_pass[i],
                      (unsigned long)g_campaign_fail[i], (unsigned long)g_campaign_warn[i],
                      (unsigned long)g_campaign_skip[i]);
    }
}

static void st_handle_uart_rx(void)
{
    uint8_t ch = 0U;
    while (HAL_UART_Receive(&ST_DBG_UART, &ch, 1U, 0U) == HAL_OK) {
        if (ch == '\r' || ch == '\n') {
            if (g_rt.cli_len == 0U) {
                st_prompt();
                continue;
            }
            st_dbg_printf("\r\n");
            g_rt.cli_buf[g_rt.cli_len] = '\0';
            st_process_line((char *)g_rt.cli_buf);
            g_rt.cli_len = 0U;
            memset(g_rt.cli_buf, 0, sizeof(g_rt.cli_buf));
            if (g_rt.mode == RUN_IDLE) st_prompt();
        } else if (ch == 0x08U || ch == 0x7FU) {
            if (g_rt.cli_len > 0U) {
                g_rt.cli_len--;
                st_dbg_printf("\b \b");
            }
        } else if (isprint((unsigned char)ch)) {
            if (g_rt.cli_len < (ST_CLI_BUF_LEN - 1U)) {
                g_rt.cli_buf[g_rt.cli_len++] = ch;
                HAL_UART_Transmit(&ST_DBG_UART, &ch, 1U, 10U);
            }
        }
    }
}

static void st_process_line(char *line)
{
    char *argv[8] = {0};
    int argc = 0;
    char *tok = strtok(line, " \t");
    while (tok != NULL && argc < (int)(sizeof(argv)/sizeof(argv[0]))) {
        argv[argc++] = tok;
        tok = strtok(NULL, " \t");
    }
    if (argc == 0) return;

    if (st_str_eq_nocase(argv[0], "help")) {
        st_print_help();
    } else if (st_str_eq_nocase(argv[0], "list")) {
        st_print_tests();
    } else if (st_str_eq_nocase(argv[0], "status")) {
        st_print_status();
    } else if (st_str_eq_nocase(argv[0], "summary")) {
        st_print_summary();
    } else if (st_str_eq_nocase(argv[0], "clear")) {
        st_reset_all_tests();
        g_rt.loop_completed_count = 0U;
        g_rt.run_fail_count = 0U;
        g_rt.run_warn_count = 0U;
        memset(g_campaign_pass, 0, sizeof(g_campaign_pass));
        memset(g_campaign_fail, 0, sizeof(g_campaign_fail));
        memset(g_campaign_warn, 0, sizeof(g_campaign_warn));
        memset(g_campaign_skip, 0, sizeof(g_campaign_skip));
        st_dbg_printf("[CLEAR] test statuses reset\r\n");
    } else if (st_str_eq_nocase(argv[0], "stop")) {
        st_abort(true);
    } else if (st_str_eq_nocase(argv[0], "run") && argc >= 2) {
        bool fast = false;
        bool both = false;
        for (int i = 2; i < argc; ++i) {
            if (st_str_eq_nocase(argv[i], "fast")) fast = true;
            else if (st_str_eq_nocase(argv[i], "both")) both = true;
        }
        g_rt.loop_fast = fast;

        if (st_str_eq_nocase(argv[1], "all")) {
            g_rt.campaign_active = false;
            st_start_all();
        } else {
            int idx = st_test_index_from_name(argv[1]);
            if (idx >= 0) {
                g_rt.campaign_active = false;
                st_start_one(g_tests[idx].mask);
            } else {
                st_dbg_printf("bad test name\r\n");
            }
        }
        if (both) {
            int sel = st_mcan_selector_from_name(argv[1]);
            if (sel < 0)
                st_dbg_printf("[MCAN] no remote mapping for %s\r\n", argv[1]);
            else
                (void)st_mcan_send_selftest_req(&g_mcan, (uint16_t)sel, fast ? 0x0001U : 0U);
        }
    } else if ((st_str_eq_nocase(argv[0], "enable") || st_str_eq_nocase(argv[0], "disable")) && argc >= 2) {
        bool en = st_str_eq_nocase(argv[0], "enable");
        if (st_str_eq_nocase(argv[1], "all")) {
            g_rt.enabled_mask = en ? ST_TEST_ALL : 0U;
        } else {
            int idx = st_test_index_from_name(argv[1]);
            if (idx >= 0) {
                if (en) g_rt.enabled_mask |= g_tests[idx].mask;
                else g_rt.enabled_mask &= ~g_tests[idx].mask;
            } else {
                st_dbg_printf("bad test name\r\n");
                return;
            }
        }
        st_dbg_printf("[%s] ok\r\n", en ? "ENABLE" : "DISABLE");
    } else if (st_str_eq_nocase(argv[0], "loop") && argc >= 2) {
        if (st_str_eq_nocase(argv[1], "on")) {
            g_rt.loop_enabled = true;
            g_rt.campaign_active = true;
            g_rt.loop_target_count = 0U;
            g_rt.loop_completed_count = 0U;
            memset(g_campaign_pass, 0, sizeof(g_campaign_pass));
            memset(g_campaign_fail, 0, sizeof(g_campaign_fail));
            memset(g_campaign_warn, 0, sizeof(g_campaign_warn));
            memset(g_campaign_skip, 0, sizeof(g_campaign_skip));
            if (g_rt.mode == RUN_IDLE) st_start_all();
            st_dbg_printf("[LOOP] on\r\n");
        } else if (st_str_eq_nocase(argv[1], "off")) {
            g_rt.loop_enabled = false;
            g_rt.campaign_active = false;
            g_rt.loop_target_count = 0U;
            st_dbg_printf("[LOOP] off\r\n");
        } else if (st_str_eq_nocase(argv[1], "count") && argc >= 3) {
            uint32_t count = (uint32_t)strtoul(argv[2], NULL, 0);
            bool fast = (argc >= 4 && st_str_eq_nocase(argv[3], "fast"));
            g_rt.loop_enabled = true;
            g_rt.campaign_active = true;
            g_rt.loop_target_count = count;
            g_rt.loop_completed_count = 0U;
            g_rt.loop_fast = fast;
            memset(g_campaign_pass, 0, sizeof(g_campaign_pass));
            memset(g_campaign_fail, 0, sizeof(g_campaign_fail));
            memset(g_campaign_warn, 0, sizeof(g_campaign_warn));
            memset(g_campaign_skip, 0, sizeof(g_campaign_skip));
            if (g_rt.loop_fast && g_rt.loop_delay_ms > ST_FAST_LOOP_DELAY_MS)
                g_rt.loop_delay_ms = ST_FAST_LOOP_DELAY_MS;
            if (g_rt.mode == RUN_IDLE) st_start_all();
            st_dbg_printf("[LOOP] count=%lu fast=%u\r\n",
                          (unsigned long)count, fast ? 1U : 0U);
        } else if (st_str_eq_nocase(argv[1], "delay") && argc >= 3) {
            g_rt.loop_delay_ms = (uint32_t)strtoul(argv[2], NULL, 0);
            st_dbg_printf("[LOOP] delay=%lu ms\r\n",
                          (unsigned long)g_rt.loop_delay_ms);
        } else if (st_str_eq_nocase(argv[1], "fast") && argc >= 3) {
            g_rt.loop_fast = st_str_eq_nocase(argv[2], "on");
            if (g_rt.loop_fast && g_rt.loop_delay_ms > ST_FAST_LOOP_DELAY_MS)
                g_rt.loop_delay_ms = ST_FAST_LOOP_DELAY_MS;
            st_dbg_printf("[LOOP] fast=%u\r\n", g_rt.loop_fast ? 1U : 0U);
        } else if (st_str_eq_nocase(argv[1], "stoponfail") && argc >= 3) {
            g_rt.loop_stop_on_fail = st_str_eq_nocase(argv[2], "on");
            st_dbg_printf("[LOOP] stoponfail=%u\r\n",
                          g_rt.loop_stop_on_fail ? 1U : 0U);
        } else {
            st_dbg_printf("bad loop command\r\n");
        }
    } else if (st_str_eq_nocase(argv[0], "therm") && argc >= 2) {
        if (st_str_eq_nocase(argv[1], "raw")) {
            st_therm_print_detail();
        } else if (st_str_eq_nocase(argv[1], "cal") && argc >= 3) {
            if (st_str_eq_nocase(argv[2], "show")) {
                st_dbg_printf("[THERM] cal offset=%.2fC\r\n", g_rt.therm_offset_c);
            } else if (st_str_eq_nocase(argv[2], "clear")) {
                g_rt.therm_offset_c = 0.0f;
                st_dbg_printf("[THERM] cal cleared\r\n");
            } else if (st_str_eq_nocase(argv[2], "offset") && argc >= 4) {
                g_rt.therm_offset_c = strtof(argv[3], NULL);
                st_dbg_printf("[THERM] cal offset=%.2fC\r\n", g_rt.therm_offset_c);
            } else {
                st_dbg_printf("bad therm cal command\r\n");
            }
        } else {
            st_dbg_printf("bad therm command\r\n");
        }
    } else if (st_str_eq_nocase(argv[0], "monitor") && argc >= 3) {
        bool on = st_str_eq_nocase(argv[2], "on");
        if (st_str_eq_nocase(argv[1], "interval")) {
            g_rt.monitor_interval_ms = (uint32_t)strtoul(argv[2], NULL, 0);
            st_dbg_printf("[MON] interval=%lu ms\r\n", (unsigned long)g_rt.monitor_interval_ms);
        } else if (st_str_eq_nocase(argv[1], "inputs")) {
            g_rt.monitor_inputs = on; st_dbg_printf("[MON] inputs=%u\r\n", on ? 1U : 0U);
        } else if (st_str_eq_nocase(argv[1], "therm")) {
            g_rt.monitor_therm = on; st_dbg_printf("[MON] therm=%u\r\n", on ? 1U : 0U);
        } else if (st_str_eq_nocase(argv[1], "esp")) {
            g_rt.monitor_esp = on; st_dbg_printf("[MON] esp=%u\r\n", on ? 1U : 0U);
        } else if (st_str_eq_nocase(argv[1], "imu0")) {
            g_rt.monitor_imu0 = on; st_dbg_printf("[MON] imu0=%u\r\n", on ? 1U : 0U);
        } else if (st_str_eq_nocase(argv[1], "imu1")) {
            g_rt.monitor_imu1 = on; st_dbg_printf("[MON] imu1=%u\r\n", on ? 1U : 0U);
        } else {
            st_dbg_printf("bad monitor command\r\n");
        }
    } else if (st_str_eq_nocase(argv[0], "beep") && argc >= 3) {
        uint32_t hz = (uint32_t)strtoul(argv[1], NULL, 0);
        uint32_t ms = (uint32_t)strtoul(argv[2], NULL, 0);
        st_buzzer_start(hz);
        HAL_Delay(ms);
        st_buzzer_off();
        st_dbg_printf("[BEEP] done\r\n");
    } else if (st_str_eq_nocase(argv[0], "set") && argc >= 3) {
        bool v = (strtoul(argv[2], NULL, 0) != 0U);
        if (st_str_eq_nocase(argv[1], "led0")) st_led_set(0U, v);
        else if (st_str_eq_nocase(argv[1], "led1")) st_led_set(1U, v);
        else if (st_str_eq_nocase(argv[1], "hb")) st_hb_set(v);
        else if (st_str_eq_nocase(argv[1], "eth_nrst")) st_eth_reset_set(v);
        else if (st_str_eq_nocase(argv[1], "can_pdu_stb")) st_can_pdu_stb_set(v);
        else if (st_str_eq_nocase(argv[1], "can_left_stb")) st_can_left_stb_set(v);
        else if (st_str_eq_nocase(argv[1], "can_right_stb")) st_can_right_stb_set(v);
        else if (st_str_eq_nocase(argv[1], "esp_rst")) st_esp_reset_set(v);
        else if (st_str_eq_nocase(argv[1], "imu0_ncs")) st_imu0_ncs_set(v);
        else if (st_str_eq_nocase(argv[1], "imu1_ncs")) st_imu1_ncs_set(v);
        else { st_dbg_printf("bad set target\r\n"); return; }
        st_dbg_printf("[SET] ok\r\n");
    } else if (st_str_eq_nocase(argv[0], "can") && argc >= 3) {
        FDCAN_HandleTypeDef *bus = NULL;
        const char *tag = NULL;
        if (st_str_eq_nocase(argv[1], "pdu")) { bus = &ST_CAN_PDU; tag = "PD"; }
        else if (st_str_eq_nocase(argv[1], "right")) { bus = &ST_CAN_RIGHT; tag = "RT"; }
        else if (st_str_eq_nocase(argv[1], "left")) { bus = &ST_CAN_LEFT; tag = "LF"; }
        else { st_dbg_printf("bad can bus\r\n"); return; }

        if (st_str_eq_nocase(argv[2], "loopback")) {
            bool ok = st_can_loopback_test(bus, tag);
            st_dbg_printf("[CAN %s] loopback %s\r\n", argv[1], ok ? "PASS" : "FAIL");
        } else if (st_str_eq_nocase(argv[2], "stb") && argc >= 4) {
            bool v = (strtoul(argv[3], NULL, 0) != 0U);
            if (bus == &ST_CAN_PDU) st_can_pdu_stb_set(v);
            else if (bus == &ST_CAN_RIGHT) st_can_right_stb_set(v);
            else st_can_left_stb_set(v);
            st_dbg_printf("[CAN %s] stb=%u\r\n", argv[1], v ? 1U : 0U);
        } else {
            st_dbg_printf("bad can command\r\n");
        }
    } else if (st_str_eq_nocase(argv[0], "eth") && argc >= 2) {
        uint32_t phy=0,id1=0,id2=0;
        if (st_str_eq_nocase(argv[1], "reset")) {
            st_eth_reset_set(true); HAL_Delay(20); st_eth_reset_set(false); HAL_Delay(100);
            st_dbg_printf("[ETH] reset pulse sent\r\n");
        } else if (st_str_eq_nocase(argv[1], "id")) {
            if (st_eth_phy_detect(&phy,&id1,&id2))
                st_dbg_printf("[ETH] PHY@%lu ID1=0x%04lX ID2=0x%04lX\r\n", (unsigned long)phy, (unsigned long)id1, (unsigned long)id2);
            else
                st_dbg_printf("[ETH] no PHY detected\r\n");
        } else if (st_str_eq_nocase(argv[1], "regs") || st_str_eq_nocase(argv[1], "link")) {
            if (st_eth_phy_detect(&phy,&id1,&id2)) st_eth_print_basic_status(phy);
            else st_dbg_printf("[ETH] no PHY detected\r\n");
        } else if (st_str_eq_nocase(argv[1], "up")) {
            eth_stack_init();
        } else if (st_str_eq_nocase(argv[1], "down")) {
            eth_stack_stop();
        } else if (st_str_eq_nocase(argv[1], "ip")) {
            eth_stack_print_ip();
        } else if (st_str_eq_nocase(argv[1], "stats")) {
            eth_stack_print_stats();
        } else if (st_str_eq_nocase(argv[1], "udp") && argc >= 3) {
            bool v = (argv[2][0] == '1' || st_str_eq_nocase(argv[2], "on"));
            eth_stack_udp_echo_set(v);
        } else {
            st_dbg_printf("bad eth command\r\n");
        }
    } else if ((st_str_eq_nocase(argv[0], "imu0") || st_str_eq_nocase(argv[0], "imu1")) && argc >= 2) {
        uint8_t idx = st_str_eq_nocase(argv[0], "imu0") ? 0U : 1U;
        uint8_t who = 0U;
        st_imu_sample_t sample;

        if (st_str_eq_nocase(argv[1], "whoami")) {
            if (st_imu_prepare(idx) && st_imu_read_whoami_idx(idx, &who)) {
                st_dbg_printf("[%s] WHO=0x%02X\r\n", st_imu_name(idx), who);
            } else {
                st_dbg_printf("[%s] read failed\r\n", st_imu_name(idx));
            }
        } else if (st_str_eq_nocase(argv[1], "read")) {
            if (!st_imu_prepare(idx) || !st_imu_read_whoami_idx(idx, &who)) {
                st_dbg_printf("[%s] WHO_AM_I read failed\r\n", st_imu_name(idx));
            } else if (who != ST_LSM6DSOX_WHOAMI) {
                st_dbg_printf("[%s] WHO=0x%02X expected 0x%02X\r\n", st_imu_name(idx), who, ST_LSM6DSOX_WHOAMI);
            } else if (st_imu_read_sample_idx(idx, true, &sample, NULL)) {
                st_imu_print_sample_idx(idx, who, &sample);
            } else {
                st_dbg_printf("[%s] sample read failed\r\n", st_imu_name(idx));
            }
        } else if (st_str_eq_nocase(argv[1], "live") && argc >= 3) {
            bool on = st_str_eq_nocase(argv[2], "on");
            if (idx == 0U) g_rt.monitor_imu0 = on;
            else g_rt.monitor_imu1 = on;
            st_dbg_printf("[%s] live=%u\r\n", st_imu_name(idx), on ? 1U : 0U);
        } else {
            st_dbg_printf("bad %s command\r\n", st_imu_name(idx));
        }
    } else if (st_str_eq_nocase(argv[0], "expect") && argc >= 3) {
        bool present = st_str_eq_nocase(argv[2], "present");
        if (st_str_eq_nocase(argv[1], "imu1")) {
            g_rt.expect_imu1_present = present;
            st_dbg_printf("[EXPECT] imu1=%s\r\n", present ? "present" : "absent");
        } else if (st_str_eq_nocase(argv[1], "esp")) {
            g_rt.expect_esp_present = present;
            st_dbg_printf("[EXPECT] esp=%s\r\n", present ? "present" : "absent");
        } else {
            st_dbg_printf("bad expect target\r\n");
        }
    } else if (st_str_eq_nocase(argv[0], "esp") && argc >= 2) {
        if (st_str_eq_nocase(argv[1], "reset")) {
            st_esp_reset_set(true); HAL_Delay(50); st_esp_reset_set(false);
            st_dbg_printf("[ESP] reset pulse sent\r\n");
        } else if (st_str_eq_nocase(argv[1], "status")) {
            st_dbg_printf("[ESP] STATUS=%u bytes_seen=%u\r\n",
                          HAL_GPIO_ReadPin(ESP_STATUS_GPIO_Port, ESP_STATUS_Pin),
                          g_rt.esp_uart_bytes_seen);
        } else if (st_str_eq_nocase(argv[1], "holdreset") && argc >= 3) {
            bool on = st_str_eq_nocase(argv[2], "on");
            g_rt.esp_hold_reset = on;
            st_esp_reset_set(on);
            st_dbg_printf("[ESP] holdreset=%u\r\n", on ? 1U : 0U);
        } else {
            st_dbg_printf("bad esp command\r\n");
        }
    } else if (st_str_eq_nocase(argv[0], "exp") && argc >= 2) {
        if (st_str_eq_nocase(argv[1], "gpio")) {
            st_dbg_printf("[EXP] GPIO=%u%u%u%u\r\n",
                          HAL_GPIO_ReadPin(EXP_GPIO0_GPIO_Port, EXP_GPIO0_Pin),
                          HAL_GPIO_ReadPin(EXP_GPIO1_GPIO_Port, EXP_GPIO1_Pin),
                          HAL_GPIO_ReadPin(EXP_GPIO2_GPIO_Port, EXP_GPIO2_Pin),
                          HAL_GPIO_ReadPin(EXP_GPIO3_GPIO_Port, EXP_GPIO3_Pin));
        } else if (st_str_eq_nocase(argv[1], "i2c") && argc >= 3 && st_str_eq_nocase(argv[2], "scan")) {
            st_exp_i2c_scan();
        } else {
            st_dbg_printf("bad exp command\r\n");
        }
    } else if (st_str_eq_nocase(argv[0], "mcan") && argc >= 2) {
        if (st_str_eq_nocase(argv[1], "monitor") && argc >= 3) {
            g_mcan.monitor = st_str_eq_nocase(argv[2], "on");
            if (g_mcan.monitor && !g_mcan.online) (void)st_mcan_bus_config(&g_mcan, true);
            st_dbg_printf("[MCAN] monitor=%u\r\n", g_mcan.monitor ? 1U : 0U);
        } else if (st_str_eq_nocase(argv[1], "diag")) {
            st_mcan_print_diag(&g_mcan);
        } else if (st_str_eq_nocase(argv[1], "hb") && argc >= 3) {
            if (st_str_eq_nocase(argv[2], "off")) {
                g_mcan.hb_enabled = false;
                st_dbg_printf("[MCAN] hb=0\r\n");
            } else if (st_str_eq_nocase(argv[2], "on")) {
                if (argc >= 4) g_mcan.hb_period_ms = (uint32_t)strtoul(argv[3], NULL, 0);
                if (g_mcan.hb_period_ms == 0U) g_mcan.hb_period_ms = ST_MCAN_DEFAULT_HB_MS;
                g_mcan.hb_enabled = true;
                if (!g_mcan.online) (void)st_mcan_bus_config(&g_mcan, true);
                st_dbg_printf("[MCAN] hb=1 period=%lu ms\r\n", (unsigned long)g_mcan.hb_period_ms);
            } else {
                st_dbg_printf("bad mcan hb command\r\n");
            }
        } else if (st_str_eq_nocase(argv[1], "ping") && argc >= 3) {
            if (st_str_eq_nocase(argv[2], "once")) {
                st_mcan_start_ping(&g_mcan, 1U, ST_MCAN_DEFAULT_PING_MS, false, true);
                st_dbg_printf("[MCAN] ping once\r\n");
            } else if (st_str_eq_nocase(argv[2], "count") && argc >= 5) {
                uint32_t count = (uint32_t)strtoul(argv[3], NULL, 0);
                uint32_t period = (uint32_t)strtoul(argv[4], NULL, 0);
                st_mcan_start_ping(&g_mcan, count, period, false, false);
                st_dbg_printf("[MCAN] ping count=%lu period=%lu ms\r\n", (unsigned long)count, (unsigned long)period);
            } else if (st_str_eq_nocase(argv[2], "cont") && argc >= 4) {
                uint32_t period = (uint32_t)strtoul(argv[3], NULL, 0);
                st_mcan_start_ping(&g_mcan, 0U, period, false, false);
                st_dbg_printf("[MCAN] ping cont period=%lu ms\r\n", (unsigned long)period);
            } else {
                st_dbg_printf("bad mcan ping command\r\n");
            }
        } else if (st_str_eq_nocase(argv[1], "pingboth") && argc >= 3) {
            if (st_str_eq_nocase(argv[2], "off")) {
                st_mcan_stop_ping(&g_mcan);
                st_dbg_printf("[MCAN] pingboth off\r\n");
            } else if (st_str_eq_nocase(argv[2], "on")) {
                uint32_t period = (argc >= 4) ? (uint32_t)strtoul(argv[3], NULL, 0) : ST_MCAN_DEFAULT_PING_MS;
                st_mcan_start_ping(&g_mcan, 0U, period, true, false);
                st_dbg_printf("[MCAN] pingboth on period=%lu ms\r\n", (unsigned long)period);
            } else {
                st_dbg_printf("bad mcan pingboth command\r\n");
            }
        } else if (st_str_eq_nocase(argv[1], "status") && argc >= 3 && st_str_eq_nocase(argv[2], "req")) {
            uint8_t seq = ++g_mcan.next_seq;
            if (st_mcan_send_simple(&g_mcan, ST_MCAN_TYPE_STATUS_REQ, seq, 0U, 0U, 0U)) {
                g_mcan.tx_req++;
                st_dbg_printf("[MCAN] status req seq=%u\r\n", seq);
            } else st_dbg_printf("[MCAN] status req failed\r\n");
        } else if (st_str_eq_nocase(argv[1], "led") && argc >= 4) {
            uint16_t count = (uint16_t)strtoul(argv[2], NULL, 0);
            uint16_t dwell = (uint16_t)strtoul(argv[3], NULL, 0);
            uint8_t seq = ++g_mcan.next_seq;
            if (st_mcan_send_simple(&g_mcan, ST_MCAN_TYPE_LED_REQ, seq, 0U, count, dwell)) {
                g_mcan.tx_req++;
                st_dbg_printf("[MCAN] led req seq=%u count=%u dwell=%u\r\n", seq, count, dwell);
            } else st_dbg_printf("[MCAN] led req failed\r\n");
        } else if (st_str_eq_nocase(argv[1], "beep") && argc >= 4) {
            uint16_t hz = (uint16_t)strtoul(argv[2], NULL, 0);
            uint16_t ms = (uint16_t)strtoul(argv[3], NULL, 0);
            uint8_t seq = ++g_mcan.next_seq;
            if (st_mcan_send_simple(&g_mcan, ST_MCAN_TYPE_BEEP_REQ, seq, 0U, hz, ms)) {
                g_mcan.tx_req++;
                st_dbg_printf("[MCAN] beep req seq=%u hz=%u ms=%u\r\n", seq, hz, ms);
            } else st_dbg_printf("[MCAN] beep req failed\r\n");
        } else if (st_str_eq_nocase(argv[1], "selftest") && argc >= 3) {
            int sel = st_mcan_selector_from_name(argv[2]);
            bool fast = (argc >= 4 && st_str_eq_nocase(argv[3], "fast"));
            if (sel < 0)
                st_dbg_printf("bad mcan selftest target\r\n");
            else
                (void)st_mcan_send_selftest_req(&g_mcan, (uint16_t)sel, fast ? 0x0001U : 0U);
        } else if (st_str_eq_nocase(argv[1], "stats")) {
            st_mcan_print_stats(&g_mcan);
        } else if (st_str_eq_nocase(argv[1], "clear")) {
            st_mcan_clear_stats(&g_mcan);
            uint8_t seq = ++g_mcan.next_seq;
            if (st_mcan_send_simple(&g_mcan, ST_MCAN_TYPE_CLEAR_STATS_REQ, seq, 0U, 0U, 0U)) {
                g_mcan.tx_req++;
                st_dbg_printf("[MCAN] local+remote clear req seq=%u\r\n", seq);
            } else {
                st_dbg_printf("[MCAN] local clear only\r\n");
            }
        } else if (st_str_eq_nocase(argv[1], "enable")) {
            if (!g_mcan.online) (void)st_mcan_bus_config(&g_mcan, true);
            st_dbg_printf("[MCAN] bus %s\r\n", g_mcan.online ? "enabled" : "enable failed");
        } else if (st_str_eq_nocase(argv[1], "reset")) {
            g_mcan.monitor = false;
            g_mcan.hb_enabled = false;
            st_mcan_stop_ping(&g_mcan);
            st_mcan_clear_stats(&g_mcan);
            st_mcan_bus_disable(&g_mcan);
            g_mcan.peer_online = false;
            g_mcan.last_peer_seen_ms = 0U;
            st_dbg_printf("[MCAN] reset\r\n");
        } else {
            st_dbg_printf("bad mcan command\r\n");
        }
    } else if (st_str_eq_nocase(argv[0], "stream") && argc >= 2) {
        if (st_str_eq_nocase(argv[1], "on")) {
            if (argc >= 3) {
                uint32_t ms = (uint32_t)strtoul(argv[2], NULL, 0);
                if (ms >= 50U) g_stream_interval_ms = ms;
            }
            g_stream_on = true;
            g_stream_next_ms = HAL_GetTick();
            st_dbg_printf("[STREAM] on interval=%lu ms\r\n",
                          (unsigned long)g_stream_interval_ms);
        } else if (st_str_eq_nocase(argv[1], "off")) {
            g_stream_on = false;
            st_dbg_printf("[STREAM] off\r\n");
        } else {
            st_dbg_printf("stream on|off [ms]\r\n");
        }
    } else {
        st_dbg_printf("unknown command\r\n");
    }
}

/* -------- tests -------- */

static void test_gpio_reset(void) { memset(&g_gpio_test, 0, sizeof(g_gpio_test)); }
static void test_gpio_start(void) { memset(&g_gpio_test, 0, sizeof(g_gpio_test)); g_gpio_test.next_ms = HAL_GetTick(); }
static st_status_t test_gpio_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    if ((int32_t)(now_ms - g_gpio_test.next_ms) < 0) return STS_RUNNING;
    switch (g_gpio_test.phase) {
        case 0: st_led_set(0, true); st_led_set(1, false); g_gpio_test.phase++; g_gpio_test.next_ms = now_ms + (g_rt.loop_fast ? 40U : 200U); return STS_RUNNING;
        case 1: st_led_set(0, false); st_led_set(1, true); g_gpio_test.phase++; g_gpio_test.next_ms = now_ms + (g_rt.loop_fast ? 40U : 200U); return STS_RUNNING;
        case 2: st_led_set(0, true); st_led_set(1, true); g_gpio_test.phase++; g_gpio_test.next_ms = now_ms + (g_rt.loop_fast ? 40U : 200U); return STS_RUNNING;
        default:
            st_led_all_off();
            snprintf(detail, detail_len, "LED walk complete");
            return STS_PASS;
    }
}

static void test_buzzer_reset(void) { memset(&g_buzzer_test, 0, sizeof(g_buzzer_test)); st_buzzer_off(); }
static void test_buzzer_start(void) { memset(&g_buzzer_test, 0, sizeof(g_buzzer_test)); g_buzzer_test.next_ms = HAL_GetTick(); }
static st_status_t test_buzzer_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    if ((int32_t)(now_ms - g_buzzer_test.next_ms) < 0) return STS_RUNNING;
    switch (g_buzzer_test.phase) {
        case 0: st_buzzer_start(1200); g_buzzer_test.phase++; g_buzzer_test.next_ms = now_ms + (g_rt.loop_fast ? 50U : 120U); return STS_RUNNING;
        case 1: st_buzzer_off();       g_buzzer_test.phase++; g_buzzer_test.next_ms = now_ms + (g_rt.loop_fast ? 20U : 60U);  return STS_RUNNING;
        case 2: st_buzzer_start(1800); g_buzzer_test.phase++; g_buzzer_test.next_ms = now_ms + (g_rt.loop_fast ? 50U : 120U); return STS_RUNNING;
        case 3: st_buzzer_off();       g_buzzer_test.phase++; g_buzzer_test.next_ms = now_ms + (g_rt.loop_fast ? 20U : 60U);  return STS_RUNNING;
        case 4: st_buzzer_start(2400); g_buzzer_test.phase++; g_buzzer_test.next_ms = now_ms + (g_rt.loop_fast ? 70U : 160U); return STS_RUNNING;
        default:
            st_buzzer_off();
            snprintf(detail, detail_len, "3-tone complete");
            return STS_PASS;
    }
}

static void test_therm_reset(void) {}
static void test_therm_start(void) {}
static st_status_t test_therm_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    (void)now_ms;
    uint16_t raw=0;
    if (!st_therm_read_raw(&raw)) { snprintf(detail, detail_len, "ADC read failed"); return STS_FAIL; }
    float v = st_adc_raw16_to_v(raw);
    float t_raw = st_therm_to_c(raw);
    float t = st_therm_apply_cal(t_raw);
    snprintf(detail, detail_len, "raw=%u V=%.4f Traw=%.2fC T=%.2fC", raw, v, t_raw, t);
    if (isnan(t)) return STS_FAIL;
    if (t < -20.0f || t > 90.0f) return STS_FAIL;
    if (t < 0.0f || t > 60.0f) return STS_WARN;
    return STS_PASS;
}

static void test_pdufault_reset(void) {}
static void test_pdufault_start(void) {}
static st_status_t test_pdufault_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    (void)now_ms;
    uint32_t v = HAL_GPIO_ReadPin(PDU_FAULT_GPIO_Port, PDU_FAULT_Pin);
    snprintf(detail, detail_len, "PDU_FAULT=%lu", (unsigned long)v);
    return v ? STS_WARN : STS_PASS;
}

static void test_canpdu_reset(void) {}
static void test_canpdu_start(void) { st_can_pdu_stb_set(true); }
static st_status_t test_canpdu_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    (void)now_ms;
    bool ok = st_can_loopback_test(&ST_CAN_PDU, "PD");
    snprintf(detail, detail_len, "%s", ok ? "internal loopback OK" : "internal loopback failed");
    return ok ? STS_PASS : STS_FAIL;
}

static void test_canleft_reset(void) {}
static void test_canleft_start(void) { st_can_left_stb_set(true); }
static st_status_t test_canleft_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    (void)now_ms;
    bool ok = st_can_loopback_test(&ST_CAN_LEFT, "LF");
    snprintf(detail, detail_len, "%s", ok ? "internal loopback OK" : "internal loopback failed");
    return ok ? STS_PASS : STS_FAIL;
}

static void test_canright_reset(void) {}
static void test_canright_start(void) { st_can_right_stb_set(true); }
static st_status_t test_canright_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    (void)now_ms;
    bool ok = st_can_loopback_test(&ST_CAN_RIGHT, "RT");
    snprintf(detail, detail_len, "%s", ok ? "internal loopback OK" : "internal loopback failed");
    return ok ? STS_PASS : STS_FAIL;
}

static void test_eth_reset(void)  { memset(&g_eth_test, 0, sizeof(g_eth_test)); }
static void test_eth_start(void)
{
    memset(&g_eth_test, 0, sizeof(g_eth_test));
    /* Stop the stack if up so we get a clean bring-up sequence */
    eth_stack_stop();
}
static st_status_t test_eth_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    switch (g_eth_test.phase) {
        case 0:
            /* Bring up: blocking init (PHY reset, autoneg, MAC config, Start).
             * eth_stack_init() returns early if already up. */
            eth_stack_init();
            g_eth_test.phase = 1U;
            g_eth_test.deadline_ms = now_ms + 3500U;
            return STS_RUNNING;
        case 1: {
            /* Wait for init to complete (it's blocking, so if we're here it's done) */
            uint32_t phy = 0U, id1 = 0U, id2 = 0U;
            if (eth_stack_is_up()) {
                bool have_phy = st_eth_phy_detect(&phy, &id1, &id2);
                snprintf(detail, detail_len, "up PHY@%lu ID1=0x%04lX ID2=0x%04lX",
                         (unsigned long)phy,
                         (unsigned long)(id1 & 0xFFFFU),
                         (unsigned long)(id2 & 0xFFFFU));
                (void)have_phy;
                return STS_PASS;
            }
            /* Not up — check if PHY at least present */
            if (!st_eth_phy_detect(&phy, &id1, &id2)) {
                snprintf(detail, detail_len, "no PHY detected");
                return STS_FAIL;
            }
            /* PHY present but no link yet — WARN */
            snprintf(detail, detail_len, "PHY@%lu ID1=0x%04lX ID2=0x%04lX no link",
                     (unsigned long)phy,
                     (unsigned long)(id1 & 0xFFFFU),
                     (unsigned long)(id2 & 0xFFFFU));
            return STS_WARN;
        }
        default:
            snprintf(detail, detail_len, "bad phase");
            return STS_FAIL;
    }
}

static void test_imu0_reset(void) {}
static void test_imu0_start(void) { (void)st_imu_prepare(0U); }
static st_status_t test_imu0_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    (void)now_ms;
    return st_test_imu_common(0U, true, detail, detail_len);
}

static void test_imu1_reset(void) {}
static void test_imu1_start(void) { (void)st_imu_prepare(1U); }
static st_status_t test_imu1_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    (void)now_ms;
    return st_test_imu_common(1U, g_rt.expect_imu1_present, detail, detail_len);
}

static void test_esp_reset(void) { memset(&g_esp_test, 0, sizeof(g_esp_test)); }
static void test_esp_start(void)
{
    memset(&g_esp_test, 0, sizeof(g_esp_test));
    g_esp_test.phase = 0U;
    g_esp_test.next_ms = HAL_GetTick();
    g_esp_test.status_before = (uint8_t)HAL_GPIO_ReadPin(ESP_STATUS_GPIO_Port, ESP_STATUS_Pin);
    g_esp_test.bytes_before = g_rt.esp_uart_bytes_seen;
}
static st_status_t test_esp_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    if ((int32_t)(now_ms - g_esp_test.next_ms) < 0) return STS_RUNNING;
    switch (g_esp_test.phase) {
        case 0:
            st_esp_reset_set(true);
            g_esp_test.phase = 1U;
            g_esp_test.next_ms = now_ms + 50U;
            return STS_RUNNING;
        case 1:
            st_esp_reset_set(false);
            g_esp_test.phase = 2U;
            g_esp_test.next_ms = now_ms + 1000U;
            return STS_RUNNING;
        default: {
            uint8_t status_after = (uint8_t)HAL_GPIO_ReadPin(ESP_STATUS_GPIO_Port, ESP_STATUS_Pin);
            uint32_t bytes_seen = (uint32_t)(g_rt.esp_uart_bytes_seen - g_esp_test.bytes_before);
            snprintf(detail, detail_len, "STATUS %u->%u UART bytes=%lu",
                     g_esp_test.status_before, status_after, (unsigned long)bytes_seen);
            if (bytes_seen > 0U || status_after != g_esp_test.status_before) return STS_PASS;
            return g_rt.expect_esp_present ? STS_WARN : STS_SKIPPED;
        }
    }
}

static void test_exp_reset(void) {}
static void test_exp_start(void) {}
static st_status_t test_exp_poll(uint32_t now_ms, char *detail, size_t detail_len)
{
    (void)now_ms;
    uint8_t g0 = HAL_GPIO_ReadPin(EXP_GPIO0_GPIO_Port, EXP_GPIO0_Pin);
    uint8_t g1 = HAL_GPIO_ReadPin(EXP_GPIO1_GPIO_Port, EXP_GPIO1_Pin);
    uint8_t g2 = HAL_GPIO_ReadPin(EXP_GPIO2_GPIO_Port, EXP_GPIO2_Pin);
    uint8_t g3 = HAL_GPIO_ReadPin(EXP_GPIO3_GPIO_Port, EXP_GPIO3_Pin);
    snprintf(detail, detail_len, "GPIO=%u%u%u%u  I2C scan manually via 'exp i2c scan'", g0,g1,g2,g3);
    return STS_PASS;
}

#endif /* !RCU_BUILD_MODE_RUNTIME */
