/*
 * pdu_adc.c — ADC sampling, PDU runtime build
 *
 * Peripheral ownership:
 *   hi2c4  — shared sequentially with fpga_mon (different I2C address)
 *   hadc1  — local ADC for V_SOURCE, V_BUS
 *   hadc2  — local ADC for THERM_0/1/2, I_COIL
 *
 * All access is superloop only; no concurrent I2C/ADC calls possible.
 *
 * TLA2528 protocol (from pdu_selftest_cli_v3.c reference):
 *   SINGLE_WRITE  opcode 0x08 → [opcode, reg, value]  (3 bytes TX)
 *   SINGLE_READ   opcode 0x10 → [opcode, reg] TX, then 1 byte RX
 *   Channel read: write CHANNEL_SEL reg, then receive 2 bytes (12-bit result
 *                 in upper 12 bits: raw12 = (rx[0]<<4) | (rx[1]>>4))
 */
#include "pdu_adc.h"
#include "main.h"

#include <math.h>
#include <string.h>

extern void st_dbg_printf(const char *fmt, ...);

/* -----------------------------------------------------------------------
 * Timing
 * ----------------------------------------------------------------------- */
#define PDU_ADC_INTERVAL_MS   100U   /* 10 Hz */

/* Oversampling counts — block-average within each 100 ms tick.
 * Current signals are noisy; voltage/temp are stable so need less. */
#define LADC_AVG_CURRENT    32U   /* 32 STM32 ADC samples for I_COIL */
#define LADC_AVG_VOLTAGE     4U   /* 4 samples for V_SOURCE / V_BUS */
#define LADC_AVG_THERM       4U   /* 4 samples for local NTC channels */
#define TLA_AVG_CURRENT      8U   /* 8 I2C reads for TLA current channels */

/* -----------------------------------------------------------------------
 * TLA2528 constants
 * ----------------------------------------------------------------------- */
#define TLA_ADDR_8BIT         (0x17U << 1)
#define TLA_I2C_TIMEOUT_MS    50U   /* Increased for clock stretching */

#define TLA_OP_SINGLE_READ    0x10U
#define TLA_OP_SINGLE_WRITE   0x08U
#define TLA_REG_SYSTEM_STATUS 0x00U
#define TLA_REG_DATA_CFG      0x02U
#define TLA_REG_OSR_CFG       0x03U
#define TLA_REG_OPMODE_CFG    0x04U
#define TLA_REG_PIN_CFG       0x05U
#define TLA_REG_CHANNEL_SEL   0x11U

/* -----------------------------------------------------------------------
 * Local ADC scaling constants (same values as self-test reference)
 * ----------------------------------------------------------------------- */
#define LADC_VDIV_SCALE       36.7142857f   /* (200K+5.6K)/5.6K */
#define LADC_ICOIL_SCALE       2.0f         /* 10 mΩ, ×50 amp */
#define ADC_REF_V              3.3f
#define ADC_12BIT_COUNTS       4096.0f

/* -----------------------------------------------------------------------
 * Peripheral references (CubeMX-owned)
 * ----------------------------------------------------------------------- */
extern I2C_HandleTypeDef  hi2c4;
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;

/* -----------------------------------------------------------------------
 * Module state
 * ----------------------------------------------------------------------- */
static pdu_ext_adc_t g_ext;
static pdu_ladc_t    g_local;
static uint32_t      g_next_ms;
static bool          g_ladc_logged;

/* -----------------------------------------------------------------------
 * Conversion helpers
 * ----------------------------------------------------------------------- */
static float raw12_to_vadc(uint16_t raw12)
{
    return ((float)raw12 * ADC_REF_V) / ADC_12BIT_COUNTS;
}

/* NTC β-model (β=3435 K, T0=25 °C, R_divider=10K/10K, Vref=3.3 V)
 * The divider output Vadc = Vref * Rntc / (R_fixed + Rntc).
 * Rntc/R_fixed = Vadc / (Vref - Vadc)  →  substitute into Steinhart-Hart. */
static float conv_therm(float vadc)
{
    if (vadc <= 0.0f || vadc >= (ADC_REF_V - 0.001f)) {
        return NAN;
    }
    float ratio = vadc / (ADC_REF_V - vadc);
    return (1.0f / (1.0f / 298.15f + (1.0f / 3435.0f) * logf(ratio))) - 273.15f;
}

/* -----------------------------------------------------------------------
 * I2C error recovery
 * ----------------------------------------------------------------------- */
static void i2c4_recover(void)
{
    /* Reset only the peripheral registers — do NOT call HAL_I2C_DeInit which
     * triggers MspDeInit and puts SCL/SDA GPIO into analog mode, corrupting
     * the bus.  RCC reset clears all I2C registers while leaving GPIO intact. */
    __HAL_RCC_I2C4_FORCE_RESET();
    __HAL_RCC_I2C4_RELEASE_RESET();
    HAL_I2C_Init(&hi2c4);
}

/* -----------------------------------------------------------------------
 * TLA2528 low-level helpers
 * ----------------------------------------------------------------------- */
static bool tla_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[3] = { TLA_OP_SINGLE_WRITE, reg, value };
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c4, TLA_ADDR_8BIT,
                                                    tx, sizeof(tx),
                                                    TLA_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        i2c4_recover();
        return false;
    }
    return true;
}

static bool tla_init(void)
{
    if (!tla_write_reg(TLA_REG_PIN_CFG,    0x00U)) return false;
    HAL_Delay(1);
    if (!tla_write_reg(TLA_REG_DATA_CFG,   0x00U)) return false;
    HAL_Delay(1);
    if (!tla_write_reg(TLA_REG_OSR_CFG,    0x00U)) return false;
    HAL_Delay(1);
    if (!tla_write_reg(TLA_REG_OPMODE_CFG, 0x00U)) return false;
    return true;
}

static bool tla_read_channel(uint8_t ch, uint16_t *raw12)
{
    uint8_t rx[2] = { 0U, 0U };
    if (ch > 7U) return false;
    if (!tla_write_reg(TLA_REG_CHANNEL_SEL, ch & 0x0FU)) return false;
    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(&hi2c4, TLA_ADDR_8BIT,
                                                   rx, sizeof(rx),
                                                   TLA_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        i2c4_recover();
        return false;
    }
    *raw12 = (uint16_t)(((uint16_t)rx[0] << 4) | ((uint16_t)rx[1] >> 4));
    return true;
}

/* Read TLA2528 channel n times and return the integer average.  Used for
 * noisy current channels where multiple I2C reads per tick reduce noise. */
static bool tla_read_channel_avg(uint8_t ch, uint8_t n, uint16_t *raw12)
{
    uint32_t acc = 0U;
    for (uint8_t i = 0U; i < n; ++i) {
        uint16_t s = 0U;
        if (!tla_read_channel(ch, &s)) return false;
        acc += s;
    }
    *raw12 = (uint16_t)(acc / (uint32_t)n);
    return true;
}

/* -----------------------------------------------------------------------
 * Local STM32 ADC helper
 * Mirrors st_ladc_read_one() from self-test: stops ADC, reconfigures
 * channel, starts, polls, reads.  Required on STM32G4 to clear ADSTART
 * before ConfigChannel can write DIFSEL.
 * ----------------------------------------------------------------------- */
static bool ladc_read_one(ADC_HandleTypeDef *hadc, uint32_t channel,
                          uint16_t *raw12)
{
    ADC_ChannelConfTypeDef cfg = { 0 };
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0U;

    (void)HAL_ADC_Stop(hadc);
    if (HAL_ADC_ConfigChannel(hadc, &cfg) != HAL_OK) return false;
    if (HAL_ADC_Start(hadc)               != HAL_OK) return false;
    if (HAL_ADC_PollForConversion(hadc, 10U) != HAL_OK) {
        (void)HAL_ADC_Stop(hadc);
        return false;
    }
    *raw12 = (uint16_t)(HAL_ADC_GetValue(hadc) & 0x0FFFU);
    (void)HAL_ADC_Stop(hadc);
    return true;
}

/* Reconfigure the channel once then take n back-to-back conversions and
 * return the integer average.  Used for noisy signals (current channels). */
static bool ladc_read_avg(ADC_HandleTypeDef *hadc, uint32_t channel,
                          uint8_t n, uint16_t *raw12)
{
    ADC_ChannelConfTypeDef cfg = { 0 };
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0U;

    (void)HAL_ADC_Stop(hadc);
    if (HAL_ADC_ConfigChannel(hadc, &cfg) != HAL_OK) return false;

    uint32_t acc = 0U;
    for (uint8_t i = 0U; i < n; ++i) {
        if (HAL_ADC_Start(hadc)              != HAL_OK) { (void)HAL_ADC_Stop(hadc); return false; }
        if (HAL_ADC_PollForConversion(hadc, 10U) != HAL_OK) { (void)HAL_ADC_Stop(hadc); return false; }
        acc += HAL_ADC_GetValue(hadc) & 0x0FFFU;
        (void)HAL_ADC_Stop(hadc);
    }
    *raw12 = (uint16_t)(acc / (uint32_t)n);
    return true;
}

/* -----------------------------------------------------------------------
 * Read external TLA2528
 * ----------------------------------------------------------------------- */
static bool read_ext_adc(void)
{
    static bool init_fail_logged = false;
    if (!tla_init()) {
        if (!init_fail_logged) {
            init_fail_logged = true;
            st_dbg_printf("[ExtADC] tla_init() FAILED\r\n");
        }
        return false;
    }

    uint16_t raw[PDU_EADC_NCH];

    /* Stable voltage and temperature channels — single read each */
    if (!tla_read_channel(PDU_EADC_V_VRAW,   &raw[PDU_EADC_V_VRAW]))   return false;
    if (!tla_read_channel(PDU_EADC_V_12V_SW, &raw[PDU_EADC_V_12V_SW])) return false;
    if (!tla_read_channel(PDU_EADC_V_24V_SW, &raw[PDU_EADC_V_24V_SW])) return false;
    if (!tla_read_channel(PDU_EADC_THERM1,   &raw[PDU_EADC_THERM1]))   return false;
    if (!tla_read_channel(PDU_EADC_THERM2,   &raw[PDU_EADC_THERM2]))   return false;

    /* Noisy current channels — averaged over TLA_AVG_CURRENT reads */
    if (!tla_read_channel_avg(PDU_EADC_I_VRAW_SW, TLA_AVG_CURRENT, &raw[PDU_EADC_I_VRAW_SW])) return false;
    if (!tla_read_channel_avg(PDU_EADC_I_12V_SW,  TLA_AVG_CURRENT, &raw[PDU_EADC_I_12V_SW]))  return false;
    if (!tla_read_channel_avg(PDU_EADC_I_24V_SW,  TLA_AVG_CURRENT, &raw[PDU_EADC_I_24V_SW]))  return false;

    float vadc[PDU_EADC_NCH];
    for (uint8_t i = 0U; i < PDU_EADC_NCH; ++i) {
        vadc[i] = raw12_to_vadc(raw[i]);
    }

    g_ext.v_vraw_v    = vadc[PDU_EADC_V_VRAW]    * 31.0f;
    g_ext.i_vraw_sw_a = vadc[PDU_EADC_I_VRAW_SW] * 9.47f;
    g_ext.v_12v_v     = vadc[PDU_EADC_V_12V_SW]  * 7.8f;
    g_ext.v_24v_v     = vadc[PDU_EADC_V_24V_SW]  * 14.0f;
    g_ext.i_12v_a     = vadc[PDU_EADC_I_12V_SW]  * 4.0f;
    g_ext.i_24v_a     = vadc[PDU_EADC_I_24V_SW]  * 4.0f;
    g_ext.therm1_c    = conv_therm(vadc[PDU_EADC_THERM1]);
    g_ext.therm2_c    = conv_therm(vadc[PDU_EADC_THERM2]);

    return true;
}

/* -----------------------------------------------------------------------
 * Read local STM32G474 ADC channels — best-effort: partial success still
 * updates the fields that did read, and sets valid=true if any succeeded.
 * ADC channel re-configuration (Stop → ConfigChannel → Start) works on
 * STM32G4 because HAL clears ADSTART in Stop before ConfigChannel writes.
 * ----------------------------------------------------------------------- */
static bool read_local_adc(void)
{
    uint16_t raw;
    bool any_ok = false;

    /* Thermistors — 4-sample average (slow-changing signal) */
    if (ladc_read_avg(&hadc2, ADC_CHANNEL_3,  LADC_AVG_THERM, &raw)) {
        g_local.therm0_c = conv_therm(raw12_to_vadc(raw));
        any_ok = true;
    }
    if (ladc_read_avg(&hadc2, ADC_CHANNEL_4,  LADC_AVG_THERM, &raw)) {
        g_local.therm1_c = conv_therm(raw12_to_vadc(raw));
        any_ok = true;
    }
    if (ladc_read_avg(&hadc2, ADC_CHANNEL_5,  LADC_AVG_THERM, &raw)) {
        g_local.therm2_c = conv_therm(raw12_to_vadc(raw));
        any_ok = true;
    }
    /* Voltage channels — 4-sample average (stable, minimal overhead) */
    if (ladc_read_avg(&hadc1, ADC_CHANNEL_15, LADC_AVG_VOLTAGE, &raw)) {
        g_local.v_source_v = raw12_to_vadc(raw) * LADC_VDIV_SCALE;
        any_ok = true;
    }
    if (ladc_read_avg(&hadc1, ADC_CHANNEL_12, LADC_AVG_VOLTAGE, &raw)) {
        g_local.v_bus_v = raw12_to_vadc(raw) * LADC_VDIV_SCALE;
        any_ok = true;
    }
    /* Current channel — 32-sample average (noisy signal) */
    if (ladc_read_avg(&hadc2, ADC_CHANNEL_12, LADC_AVG_CURRENT, &raw)) {
        g_local.i_coil_a = raw12_to_vadc(raw) * LADC_ICOIL_SCALE;
        any_ok = true;
    }
    return any_ok;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void pdu_adc_init(void)
{
    memset(&g_ext,   0, sizeof(g_ext));
    memset(&g_local, 0, sizeof(g_local));
    g_next_ms     = 0U;
    g_ladc_logged = false;

    /* Do NOT call HAL_ADCEx_Calibration_Start here: the self-test never calls
     * it and reads local ADC correctly straight from the CubeMX-init state.
     * Calibration is optional on STM32G4; omitting it matches the verified
     * self-test code path. */
}

void pdu_adc_tick(uint32_t now_ms)
{
    if (now_ms < g_next_ms) {
        return;
    }
    g_next_ms = now_ms + PDU_ADC_INTERVAL_MS;

    uint32_t ts = HAL_GetTick();

    if (read_ext_adc()) {
        g_ext.valid        = true;
        g_ext.last_read_ms = ts;
    } else {
        /* Hold last good data for 1000ms before invalidating */
        if (g_ext.valid && (now_ms - g_ext.last_read_ms) > 1000U) {
            g_ext.valid = false;
        }
    }

    if (read_local_adc()) {
        g_local.valid        = true;
        g_local.last_read_ms = ts;
        if (!g_ladc_logged) {
            g_ladc_logged = true;
            st_dbg_printf("[LADC] first ok: T0=%.1f T1=%.1f T2=%.1f V_SRC=%.3f\r\n",
                          (double)g_local.therm0_c, (double)g_local.therm1_c,
                          (double)g_local.therm2_c, (double)g_local.v_source_v);
        }
    } else {
        /* Hold last good data for 1000ms before invalidating */
        if (g_local.valid && (now_ms - g_local.last_read_ms) > 1000U) {
            g_local.valid = false;
        }
        if (!g_ladc_logged) {
            g_ladc_logged = true;
            /* Probe each channel individually to identify which HAL call fails */
            uint16_t raw = 0U;
            int r3  = (ladc_read_one(&hadc2, ADC_CHANNEL_3,  &raw) ? (int)raw : -1);
            int r15 = (ladc_read_one(&hadc1, ADC_CHANNEL_15, &raw) ? (int)raw : -1);
            st_dbg_printf("[LADC] read_local_adc failed: CH3=%d CH15=%d\r\n", r3, r15);
        }
    }
}

const pdu_ext_adc_t *pdu_adc_get_ext(void)
{
    return &g_ext;
}

const pdu_ladc_t *pdu_adc_get_local(void)
{
    return &g_local;
}
