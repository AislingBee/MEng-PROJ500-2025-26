/*
 * rcu_app.c — RCU runtime application
 *
 * Superloop architecture:
 *   RCU_App_Init()  — called once after all MX init
 *   RCU_App_Task()  — called every superloop pass (no blocking)
 *
 * Task sequence per iteration:
 *   1. mcan_pdu_tick    — drain MCAN PDU bus, send heartbeat
 *   2. imu_tick         — service any pending IMU DRDY flags
 *   3. motor_bus_tick   — drain motor CAN buses, flush queued commands
 *   4. Slow-telem TX    — every 100 ms  (10 Hz, adjustable)
 *   5. Fast-loop TX     — every 5 ms    (200 Hz): IMU fast pkt + motor FB
 *   6. eth_udp_tick     — MX_LWIP_Process() + drain Rx
 *   7. led_heartbeat    — LED_1 blinks at 1 Hz
 */
#include "rcu_app.h"
#include "main.h"
#include "mcan_pdu.h"
#include "imu.h"
#include "motor_bus.h"
#include "eth_udp.h"
#include "telem_pack.h"

extern void st_dbg_printf(const char *fmt, ...);
extern TIM_HandleTypeDef htim1;

/* -----------------------------------------------------------------------
 * Timing constants (ms)
 * ----------------------------------------------------------------------- */
#define SLOW_TELEM_MS    100U   /* 10 Hz */
#define FAST_LOOP_MS       5U   /* 200 Hz: fast IMU + motor FB */
#define LED_BLINK_MS     500U   /* 1 Hz toggle */

/* -----------------------------------------------------------------------
 * Module state
 * ----------------------------------------------------------------------- */
static uint32_t g_next_slow_telem;
static uint32_t g_next_fast_loop;
static uint32_t g_next_led_blink;
static bool     g_led_state;

static volatile uint32_t g_slow_telem_ms = SLOW_TELEM_MS;

void rcu_app_set_telem_rate_ms(uint32_t ms)
{
    if (ms < 50U)    ms = 50U;
    if (ms > 1000U)  ms = 1000U;
    g_slow_telem_ms    = ms;
    g_next_slow_telem  = HAL_GetTick();   /* take effect immediately */
}

/* -----------------------------------------------------------------------
 * Hardware watchdog timer (TIM6) — toggles MCU_HB independently of the
 * superloop so a stalled loop cannot miss the MCP1316 WDI window.
 * ----------------------------------------------------------------------- */
static TIM_HandleTypeDef s_wdog_tim;

static void wdog_timer_init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* Compute TIM6 kernel clock dynamically to survive any CubeMX
     * clock-tree changes.  TIM6 is on APB1; if the APB1 prescaler
     * is not /1 the timer clock is doubled per the H7 reference manual. */
    RCC_ClkInitTypeDef clk_init = {0};
    uint32_t flash_latency = 0U;
    HAL_RCC_GetClockConfig(&clk_init, &flash_latency);
    uint32_t pclk1   = HAL_RCC_GetPCLK1Freq();
    uint32_t tim_clk = (clk_init.APB1CLKDivider == RCC_APB1_DIV1)
                       ? pclk1 : (pclk1 * 2U);

    /* Prescale to 10 kHz, count 200 ticks -> 20 ms period (50 Hz toggle).
     * 20 ms is well inside the 1.12 s minimum WDT window of MCP1316MT. */
    s_wdog_tim.Instance               = TIM6;
    s_wdog_tim.Init.Prescaler         = (tim_clk / 10000U) - 1U;
    s_wdog_tim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_wdog_tim.Init.Period            = 199U;
    s_wdog_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&s_wdog_tim);

    /* Priority 2 — must fire even when EXTI/FDCAN ISRs are active.
     * WDI window is 1.12 s min; 200 ms period gives 5× margin, but only
     * if this ISR is not permanently blocked by a lower-numbered priority. */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    HAL_TIM_Base_Start_IT(&s_wdog_tim);
    HAL_GPIO_WritePin(MCU_HB_GPIO_Port, MCU_HB_Pin, GPIO_PIN_SET);
}

/* IRQ handler defined here to keep WDT logic self-contained.
 * TIM6 is a basic timer: the only interrupt source is the update event. */
void TIM6_DAC_IRQHandler(void)
{
    __HAL_TIM_CLEAR_IT(&s_wdog_tim, TIM_IT_UPDATE);
    HAL_GPIO_TogglePin(MCU_HB_GPIO_Port, MCU_HB_Pin);
    /* LED_0 is driven by superloop at 1 Hz — do NOT toggle here */
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void RCU_App_Init(void)
{
    st_dbg_printf("\r\n=== RCU Runtime v1 ===\r\n");

    /* ------------------------------------------------------------------
     * Log the reset cause from RCC_RSR so we can tell apart:
     *   PINRSTF  = external NRST (supervisor WDT fired, or manual reset)
     *   SFTRSTF  = NVIC_SystemReset() (fault handler / Error_Handler fired)
     *   BORRSTF  = brownout
     *   IWDGxRST = internal watchdog (should not happen)
     * ------------------------------------------------------------------ */
    {
        uint32_t rsr = RCC->RSR;
        st_dbg_printf("[RCU] Reset cause RSR=0x%08lX:", (unsigned long)rsr);
        if (rsr & RCC_RSR_PINRSTF)   st_dbg_printf(" NRST(supervisor/button)");
        if (rsr & RCC_RSR_SFTRSTF)   st_dbg_printf(" SW-RESET(fault/error)");
        if (rsr & RCC_RSR_BORRSTF)   st_dbg_printf(" BOR(brownout)");
        if (rsr & RCC_RSR_IWDG1RSTF) st_dbg_printf(" IWDG1");
        if (rsr & RCC_RSR_WWDG1RSTF) st_dbg_printf(" WWDG1");
        if (rsr & RCC_RSR_D1RSTF)    st_dbg_printf(" D1-PWR");
        if (rsr & RCC_RSR_D2RSTF)    st_dbg_printf(" D2-PWR");
        st_dbg_printf("\r\n");
        /* Clear all reset flags so the next reboot shows a fresh cause. */
        SET_BIT(RCC->RSR, RCC_RSR_RMVF);
    }

    /* Subsystem init — order matters: MCAN and motor buses start here. */
    mcan_pdu_init();
    st_dbg_printf("[RCU] MCAN-PDU init OK\r\n");

    bool imu_ok = imu_init();
    st_dbg_printf("[RCU] IMU init %s\r\n", imu_ok ? "OK" : "FAIL (IMU not detected)");

    motor_bus_init();
    st_dbg_printf("[RCU] Motor buses init OK\r\n");

    eth_udp_init();      /* must be after MX_LWIP_Init() in main.c */
    st_dbg_printf("[RCU] UDP/lwIP init OK\r\n");

    /* Brief 80 ms boot tone — 1500 Hz on TIM1 CH1/CH1N (PE9/PE8) */
    __HAL_TIM_SET_AUTORELOAD(&htim1, 33332U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 16666U);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_Delay(80U);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

    /* Start hardware WDT toggle — must be last so no long init code follows
     * that could delay the first edge beyond the WDT window on next boot. */
    wdog_timer_init();
    st_dbg_printf("[RCU] Init complete — entering superloop\r\n");

    /* Initial LED state — use LED_0 (green, PC0) for heartbeat */
    g_led_state = false;
    HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);

    /* Stagger first transmissions */
    uint32_t now = HAL_GetTick();
    g_next_slow_telem = now + SLOW_TELEM_MS;
    g_next_fast_loop  = now + FAST_LOOP_MS;
    g_next_led_blink  = now + LED_BLINK_MS;
}

void RCU_App_Task(void)
{
    uint32_t now = HAL_GetTick();

    /* 1. MCAN PDU */
    mcan_pdu_tick(now);

    /* 2. IMU */
    imu_tick(now);

    /* 3. Motor buses */
    motor_bus_tick(now);

    /* 4. Slow telemetry (10 Hz) or forced by debug command */
    if (now >= g_next_slow_telem || eth_udp_consume_force_telem()) {
        g_next_slow_telem = now + g_slow_telem_ms;

        rcu_telem_payload_t tpkt;
        telem_pack_slow(&tpkt);
        eth_udp_send_telem(&tpkt);

        /* DIAG: print key validity flags and PDU CAN HB age every 2 s */
        static uint32_t s_diag_next = 0U;
        if (now >= s_diag_next) {
            s_diag_next = now + 2000U;
            const pdu_telem_t *pdu = mcan_pdu_get_telem();
            uint32_t hb_age = (pdu->hb_last_ms > 0U) ? (now - pdu->hb_last_ms) : 99999U;
            uint32_t fpga_age = (pdu->fpga_last_ms > 0U) ? (now - pdu->fpga_last_ms) : 99999U;
            uint32_t rails_age = (pdu->rails_last_ms > 0U) ? (now - pdu->rails_last_ms) : 99999U;
            st_dbg_printf("[DIAG] telem TX: fpga_v=%d(age=%lums) rails_v=%d(age=%lums) "
                          "ssd_v=%d local_v=%d  pdu_hb_age=%lums  dma_rst=%lu\r\n",
                          (int)pdu->fpga_valid,  (unsigned long)fpga_age,
                          (int)pdu->rails_valid, (unsigned long)rails_age,
                          (int)pdu->ssd_valid,   (int)pdu->local_valid,
                          (unsigned long)hb_age,
                          (unsigned long)eth_udp_get_dma_resets());
        }
    }

    /* 5. Fast loop (200 Hz): fast IMU packet + motor feedback */
    if (now >= g_next_fast_loop) {
        g_next_fast_loop = now + FAST_LOOP_MS;

        rcu_imu_fast_t ipkt;
        telem_pack_imu_fast(&ipkt);
        eth_udp_send_imu_fast(&ipkt);

        rcu_motor_fb_payload_t mpkt;
        telem_pack_motor_fb(&mpkt);
        eth_udp_send_motor_fb(&mpkt);
    }

    /* 6. lwIP + Rx dispatch */
    eth_udp_tick(now);

    /* 7. LED_0 heartbeat blink (green LED, PC0) */
    if (now >= g_next_led_blink) {
        g_next_led_blink = now + LED_BLINK_MS;
        g_led_state = !g_led_state;
        HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin,
                          g_led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    /* 8. MCU_HB watchdog toggle is handled by TIM6_DAC_IRQHandler at 5 Hz.
     *    No superloop code required. */
}
