/**
 * STM32F4xx HAL Configuration - Nucleo CAN Bridge
 * Enable only the HAL modules we need.
 */
#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Oscillator values ──────────────────────────────────── */
#if !defined(HSE_VALUE)
#define HSE_VALUE    8000000U   /* 8 MHz HSE on Nucleo (from ST-Link) */
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT  100U
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE    16000000U
#endif

#if !defined(LSI_VALUE)
#define LSI_VALUE    32000U
#endif

#if !defined(LSE_VALUE)
#define LSE_VALUE    32768U
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT  5000U
#endif

#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE  12288000U
#endif

/* ─── HAL Module Selection ───────────────────────────────── */
#define HAL_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_CAN_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED

/* ─── SysTick interrupt priority ────────────────────────── */
#define TICK_INT_PRIORITY       0x0FU
#define USE_RTOS                0U
#define PREFETCH_ENABLE         1U
#define INSTRUCTION_CACHE_ENABLE  1U
#define DATA_CACHE_ENABLE       1U

/* ─── Include HAL headers for enabled modules ────────────── */
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_pwr_ex.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_rcc_ex.h"
#include "stm32f4xx_hal_flash_ex.h"

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CONF_H */
