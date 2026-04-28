/*
 * rcu_app.h — RCU runtime application entry points
 *
 * Dispatch targets for main.c when RCU_BUILD_MODE_RUNTIME is defined.
 */
#ifndef RCU_APP_H
#define RCU_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  One-time initialisation.  Call in main() after all MX_xxx_Init().
 */
void RCU_App_Init(void);

/**
 * @brief  Superloop body.  Call repeatedly in while(1).
 */
void RCU_App_Task(void);

/**
 * @brief  Adjust slow-telemetry interval.  Clamped to [50, 1000] ms.
 *         Takes effect immediately.  Default is 100 ms (10 Hz).
 */
void rcu_app_set_telem_rate_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* RCU_APP_H */
