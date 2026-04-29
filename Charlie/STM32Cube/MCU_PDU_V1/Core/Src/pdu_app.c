/*
 * pdu_app.c — PDU top-level runtime application
 *
 * Superloop tick order (all non-blocking except SSD which blocks for
 * ~150 ms every 200 ms when querying the energy meter):
 *
 *   1. fpga_mon_tick     — I2C4: FPGA poll (200 ms interval, ~2 ms per read)
 *   2. pdu_adc_tick      — I2C4: TLA2528 + ADC1/2 (100 ms interval)
 *                          NOTE: hi2c4 is shared with fpga_mon.  Both modules
 *                          access it sequentially here — no concurrent risk.
 *   3. ssd_energy_tick   — USART3/RS485: SSD poll (200 ms interval)
 *   4. pdu_mcan_app_tick — FDCAN1: telemetry TX + command RX
 *   5. Process commands  — act on any latched AUX / CMD_FAULT requests
 *
 * GPIO outputs set at init and maintained throughout:
 *   CMD_ARM   HIGH — arm veto removed; FPGA permits arming when safe
 *   CMD_FAULT LOW  — not in fault unless RCU commands it
 *   AUX_SW_x  LOW  — all auxiliary switches off at startup
 */
#include "pdu_app.h"
#include "main.h"

#include "fpga_mon.h"
#include "pdu_adc.h"
#include "ssd_energy.h"
#include "pdu_mcan_app.h"

extern void st_dbg_printf(const char *fmt, ...);
extern TIM_HandleTypeDef htim17;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* -----------------------------------------------------------------------
 * Periodic debug print — UART4 every 2 s.  Set to 0 to silence.
 * Prints SSD and local-ADC snapshots plus ADC HAL state registers so
 * the exact failure point in ladc_read_one() is visible.
 * ADC state bits: 0x01=READY 0x10=ERR_INTERNAL 0x20=ERR_CONFIG
 *                 0x100=REG_BUSY 0x200=REG_EOC 0x400=REG_OVR
 * ----------------------------------------------------------------------- */
#define PDU_PERIODIC_DEBUG  1

#if PDU_PERIODIC_DEBUG
static uint32_t g_dbg_next_ms;

static void pdu_debug_tick(uint32_t now_ms)
{
    if (now_ms < g_dbg_next_ms) return;
    g_dbg_next_ms = now_ms + 2000U;

    const fpga_snapshot_t *fpga = fpga_mon_get();
    const pdu_ext_adc_t   *ext  = pdu_adc_get_ext();
    const ssd_snapshot_t  *ssd  = ssd_energy_get();
    const pdu_ladc_t      *ladc = pdu_adc_get_local();

    st_dbg_printf("[DBG] FPGA valid=%d  sts0=0x%02X  fc=0x%02X  sc=%d  ver=0x%02X\r\n",
                  (int)fpga->valid,
                  (unsigned)fpga->status0,
                  (unsigned)fpga->fault_code,
                  (int)fpga->state_code,
                  (unsigned)fpga->version);

    /* FDCAN1 protocol status — catches bus-off, error-passive, LEC errors */
    {
        extern FDCAN_HandleTypeDef hfdcan1;
        uint32_t psr = hfdcan1.Instance->PSR;
        uint32_t ecr = hfdcan1.Instance->ECR;
        st_dbg_printf("[DBG] FDCAN1 PSR=0x%08lX ECR=0x%08lX  BO=%d EP=%d EW=%d TEC=%lu REC=%lu\r\n",
                      (unsigned long)psr, (unsigned long)ecr,
                      (int)((psr >> 7) & 1U),   /* Bus Off */
                      (int)((psr >> 5) & 1U),   /* Error Passive */
                      (int)((psr >> 6) & 1U),   /* Error Warning */
                      (unsigned long)((ecr >> 16) & 0xFFU),  /* TEC */
                      (unsigned long)(ecr & 0xFFU));         /* REC */
    }

    st_dbg_printf("[DBG] ExtADC valid=%d  VRAW=%.2fV  12V=%.3fV  24V=%.3fV  I_VRAW=%.3fA\r\n",
                  (int)ext->valid,
                  (double)ext->v_vraw_v,
                  (double)ext->v_12v_v,
                  (double)ext->v_24v_v,
                  (double)ext->i_vraw_sw_a);

    st_dbg_printf("[DBG] SSD  valid=%d  I=%.3fA  V=%.3fV  T=%.1fC\r\n",
                  (int)ssd->valid,
                  (double)ssd->current_a,
                  (double)ssd->voltage_v,
                  (double)ssd->temp_c);

    st_dbg_printf("[DBG] LADC valid=%d  T0=%.1f T1=%.1f T2=%.1f  V_SRC=%.3fV  V_BUS=%.3fV  I_COIL=%.3fA\r\n",
                  (int)ladc->valid,
                  (double)ladc->therm0_c,
                  (double)ladc->therm1_c,
                  (double)ladc->therm2_c,
                  (double)ladc->v_source_v,
                  (double)ladc->v_bus_v,
                  (double)ladc->i_coil_a);

    st_dbg_printf("[DBG] ADC1_st=0x%08lX  ADC2_st=0x%08lX\r\n",
                  (unsigned long)HAL_ADC_GetState(&hadc1),
                  (unsigned long)HAL_ADC_GetState(&hadc2));
}
#endif

/* -----------------------------------------------------------------------
 * Private helpers
 * ----------------------------------------------------------------------- */

static inline void led_set(uint8_t idx, bool on)
{
    GPIO_PinState s = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if (idx == 0U) HAL_GPIO_WritePin(LED_0_GPIO_Port,    LED_0_Pin,    s);
    if (idx == 1U) HAL_GPIO_WritePin(LED_1_GPIO_Port,    LED_1_Pin,    s);
}

static inline void aux_sw_set(uint8_t n, bool on)
{
    const uint16_t pins[]  = { 0, AUX_SW_1_Pin, AUX_SW_2_Pin, AUX_SW_3_Pin };
    GPIO_TypeDef  *ports[] = { NULL, AUX_SW_1_GPIO_Port, AUX_SW_2_GPIO_Port, AUX_SW_3_GPIO_Port };
    if (n < 1U || n > 3U) return;
    HAL_GPIO_WritePin(ports[n], pins[n], on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* -----------------------------------------------------------------------
 * Heartbeat LED (blinks LED_1 at 1 Hz to show runtime is running)
 * -----------------------------------------------------------------------
 * External watchdog (MCU_HB_Pin = PC1) requires a toggle every <1.6 s.
 * We toggle at 5 Hz (200 ms) for comfortable margin.
 * ----------------------------------------------------------------------- */
static uint32_t g_led_next_ms;
#define LED_BLINK_MS  500U   /* LED: 1 Hz */

static void led_heartbeat_tick(uint32_t now_ms)
{
    if (now_ms >= g_led_next_ms) {
        g_led_next_ms = now_ms + LED_BLINK_MS;
        HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);  /* green LED */
    }
}

/* -----------------------------------------------------------------------
 * Hardware watchdog timer (TIM6) — toggles MCU_HB independently of the
 * superloop so a stalled loop cannot miss the MCP1316 WDI window.
 * ----------------------------------------------------------------------- */
static TIM_HandleTypeDef s_wdog_tim;

static void wdog_timer_init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* Compute TIM6 kernel clock dynamically.  On STM32G4 the APB1 divider
     * constant is RCC_HCLK_DIV1; if APB1 is not divided, timer clock equals
     * APB1 clock, otherwise it is doubled per the G4 reference manual. */
    RCC_ClkInitTypeDef clk_init = {0};
    uint32_t flash_latency = 0U;
    HAL_RCC_GetClockConfig(&clk_init, &flash_latency);
    uint32_t pclk1   = HAL_RCC_GetPCLK1Freq();
    uint32_t tim_clk = (clk_init.APB1CLKDivider == RCC_HCLK_DIV1)
                       ? pclk1 : (pclk1 * 2U);

    /* Prescale to 10 kHz, count 2000 ticks -> 200 ms period (5 Hz toggle).
     * 200 ms is well inside the 1.12 s minimum WDT window of MCP1316MT. */
    s_wdog_tim.Instance               = TIM6;
    s_wdog_tim.Init.Prescaler         = (tim_clk / 10000U) - 1U;
    s_wdog_tim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_wdog_tim.Init.Period            = 1999U;
    s_wdog_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&s_wdog_tim);

    /* Lowest NVIC priority — WDI toggle is not latency-sensitive */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 15U, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    HAL_TIM_Base_Start_IT(&s_wdog_tim);
}

/* IRQ handler defined here to keep WDT logic self-contained.
 * TIM6 is a basic timer: the only interrupt source is the update event. */
void TIM6_DAC_IRQHandler(void)
{
    __HAL_TIM_CLEAR_IT(&s_wdog_tim, TIM_IT_UPDATE);
    HAL_GPIO_TogglePin(MCU_HB_GPIO_Port, MCU_HB_Pin);
}

static void buzzer_beep(uint32_t hz, uint32_t ms)
{
    uint32_t arr = (170000000U / (hz ? hz : 1000U)) - 1U;
    if (arr > 65535U) arr = 65535U;
    __HAL_TIM_SET_AUTORELOAD(&htim17, arr);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, arr / 2U);
    __HAL_TIM_SET_COUNTER(&htim17, 0U);
    HAL_TIM_GenerateEvent(&htim17, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);
    HAL_Delay(ms);
    HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim17, TIM_CHANNEL_1);
}

/* -----------------------------------------------------------------------
 * Command processing
 * ----------------------------------------------------------------------- */

static void process_commands(pdu_mcan_cmd_t *cmds)
{
    if (cmds->aux_cmd_pending) {
        cmds->aux_cmd_pending = false;
        uint8_t b = cmds->aux_cmd_byte;
        aux_sw_set(1U, (b & 0x01U) != 0U);
        aux_sw_set(2U, (b & 0x02U) != 0U);
        aux_sw_set(3U, (b & 0x04U) != 0U);
    }

    if (cmds->buzz_pending) {
        cmds->buzz_pending = false;
        buzzer_beep(1500U, 200U);
    }

    if (cmds->led_blink_pending) {
        cmds->led_blink_pending = false;
        for (uint8_t i = 0U; i < 3U; ++i) {
            led_set(1U, true);
            HAL_Delay(80U);
            led_set(1U, false);
            HAL_Delay(80U);
        }
    }

    if (cmds->fault_assert_pending) {
        cmds->fault_assert_pending = false;
        HAL_GPIO_WritePin(CMD_FAULT_GPIO_Port, CMD_FAULT_Pin, GPIO_PIN_SET);
        led_set(0U, true);   /* LED_0 on = fault asserted */
    }

    if (cmds->fault_clear_pending) {
        cmds->fault_clear_pending = false;
        HAL_GPIO_WritePin(CMD_FAULT_GPIO_Port, CMD_FAULT_Pin, GPIO_PIN_RESET);
        led_set(0U, false);
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void PDU_App_Init(void)
{
    st_dbg_printf("\r\n=== PDU Runtime v1 ===\r\n");

    /* Safe output state first */
    led_set(0U, false);
    led_set(1U, false);
    aux_sw_set(1U, false);
    aux_sw_set(2U, false);
    aux_sw_set(3U, false);

    /* CMD_ARM held HIGH — arm-veto feature removed per design decision */
    HAL_GPIO_WritePin(CMD_ARM_GPIO_Port,   CMD_ARM_Pin,   GPIO_PIN_SET);
    /* CMD_FAULT de-asserted at startup */
    HAL_GPIO_WritePin(CMD_FAULT_GPIO_Port, CMD_FAULT_Pin, GPIO_PIN_RESET);

    /* Subsystem init */
    fpga_mon_init();
    st_dbg_printf("[PDU] FPGA mon init OK\r\n");

    pdu_adc_init();
    st_dbg_printf("[PDU] ADC init OK\r\n");

    ssd_energy_init();
    st_dbg_printf("[PDU] SSD energy init OK\r\n");

    pdu_mcan_app_init();
    st_dbg_printf("[PDU] MCAN app init OK\r\n");

    /* Brief 80 ms boot tone — 1500 Hz on TIM17 CH1/CH1N (PB5/PB7) */
    __HAL_TIM_SET_AUTORELOAD(&htim17, 31999U);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 15999U);
    __HAL_TIM_SET_COUNTER(&htim17, 0U);
    HAL_TIM_GenerateEvent(&htim17, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);
    HAL_Delay(80U);
    HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim17, TIM_CHANNEL_1);

    /* Start hardware WDT toggle — must be last so no long init code follows
     * that could delay the first edge beyond the WDT window on next boot. */
    wdog_timer_init();
    st_dbg_printf("[PDU] Init complete — entering superloop\r\n");

    g_led_next_ms  = 0U;
}

void PDU_App_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t t0, t1;

#define TASK_TIME(name, call) \
    t0 = HAL_GetTick(); call; t1 = HAL_GetTick(); \
    if ((t1 - t0) > 20U) \
        st_dbg_printf("[PDU_SLOW] " name " took %lums\r\n", (unsigned long)(t1 - t0));

    TASK_TIME("fpga_mon_tick",    fpga_mon_tick(now))
    TASK_TIME("pdu_adc_tick",     pdu_adc_tick(now))
    TASK_TIME("ssd_energy_tick",  ssd_energy_tick(now))
    /* Refresh timestamp: ssd_energy_tick blocks ~320 ms (RS485 turnaround).
     * Without this, pdu_mcan_app_tick receives a stale 'now' and the CAN
     * heartbeat and telemetry timers fire late (HB at ~1.4 Hz, telem ~2.8 Hz
     * instead of their nominal 2 Hz / 10 Hz rates). */
    now = HAL_GetTick();
    TASK_TIME("mcan_app_tick",    pdu_mcan_app_tick(now, fpga_mon_get(), pdu_adc_get_ext(), ssd_energy_get(), pdu_adc_get_local()))
    TASK_TIME("process_commands", process_commands(pdu_mcan_app_cmds()))

#undef TASK_TIME

    led_heartbeat_tick(now);

#if PDU_PERIODIC_DEBUG
    pdu_debug_tick(now);
#endif
}
