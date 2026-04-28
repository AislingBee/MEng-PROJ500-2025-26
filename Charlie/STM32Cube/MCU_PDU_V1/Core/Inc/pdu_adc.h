/*
 * pdu_adc.h — ADC sampling for PDU runtime build
 *
 * Two ADC sources are managed by this module, both accessed from the
 * superloop only (no DMA, no ISR):
 *
 *   External TLA2528 (I2C4, addr 0x17):
 *     CH0  V_VRAW    — 31× divider
 *     CH1  I_VRAW_SW — 9.47× shunt amp
 *     CH2  V_12V_SW  — 7.8× divider
 *     CH3  V_24V_SW  — 14× divider
 *     CH4  I_12V_SW  — 4× shunt amp
 *     CH5  I_24V_SW  — 4× shunt amp
 *     CH6  THERM1    — NTC β-model
 *     CH7  THERM2    — NTC β-model
 *
 *   Local STM32G474 ADC (hadc1 / hadc2):
 *     THERM_0  PA6  ADC2_CH3   — NTC β-model
 *     THERM_1  PA7  ADC2_CH4   — NTC β-model
 *     THERM_2  PC4  ADC2_CH5   — NTC β-model
 *     V_SOURCE PB0  ADC1_CH15  — 36.71× divider
 *     V_BUS    PB1  ADC1_CH12  — 36.71× divider
 *     I_COIL   PB2  ADC2_CH12  — 2× (10 mΩ, ×50 amp)
 */
#ifndef PDU_ADC_H
#define PDU_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * External ADC channel indices (for pdu_ext_adc_t arrays)
 * ----------------------------------------------------------------------- */
#define PDU_EADC_V_VRAW     0U
#define PDU_EADC_I_VRAW_SW  1U
#define PDU_EADC_V_12V_SW   2U
#define PDU_EADC_V_24V_SW   3U
#define PDU_EADC_I_12V_SW   4U
#define PDU_EADC_I_24V_SW   5U
#define PDU_EADC_THERM1     6U
#define PDU_EADC_THERM2     7U
#define PDU_EADC_NCH        8U

/* Local ADC channel indices (for pdu_ladc_t arrays) */
#define PDU_LADC_THERM_0    0U
#define PDU_LADC_THERM_1    1U
#define PDU_LADC_THERM_2    2U
#define PDU_LADC_V_SOURCE   3U
#define PDU_LADC_V_BUS      4U
#define PDU_LADC_I_COIL     5U
#define PDU_LADC_NCH        6U

/* -----------------------------------------------------------------------
 * Snapshot types
 * ----------------------------------------------------------------------- */
typedef struct {
    bool     valid;
    uint32_t last_read_ms;
    float    v_vraw_v;        /* PDU_EADC_V_VRAW   engineering [V] */
    float    i_vraw_sw_a;     /* PDU_EADC_I_VRAW_SW              [A] */
    float    v_12v_v;         /* PDU_EADC_V_12V_SW               [V] */
    float    v_24v_v;         /* PDU_EADC_V_24V_SW               [V] */
    float    i_12v_a;         /* PDU_EADC_I_12V_SW               [A] */
    float    i_24v_a;         /* PDU_EADC_I_24V_SW               [A] */
    float    therm1_c;        /* PDU_EADC_THERM1                 [°C] */
    float    therm2_c;        /* PDU_EADC_THERM2                 [°C] */
} pdu_ext_adc_t;

typedef struct {
    bool     valid;
    uint32_t last_read_ms;
    float    therm0_c;        /* PA6  [°C] */
    float    therm1_c;        /* PA7  [°C] */
    float    therm2_c;        /* PC4  [°C] */
    float    v_source_v;      /* PB0  [V] */
    float    v_bus_v;         /* PB1  [V] */
    float    i_coil_a;        /* PB2  [A] */
} pdu_ladc_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise both ADC modules.  No I2C/ADC transactions yet.
 */
void pdu_adc_init(void);

/**
 * @brief  Non-blocking poll.  Call every superloop iteration.
 *         Reads external ADC then local ADC at PDU_ADC_INTERVAL_MS.
 * @param  now_ms  Current HAL_GetTick() value.
 */
void pdu_adc_tick(uint32_t now_ms);

/**
 * @brief  Return pointer to latest external ADC snapshot.
 */
const pdu_ext_adc_t *pdu_adc_get_ext(void);

/**
 * @brief  Return pointer to latest local ADC snapshot.
 */
const pdu_ladc_t *pdu_adc_get_local(void);

#ifdef __cplusplus
}
#endif

#endif /* PDU_ADC_H */
