/*
 * pdu_app.h — PDU top-level runtime application
 *
 * Owns the superloop and all PDU peripherals in PDU_BUILD_MODE_RUNTIME.
 * Calls each subsystem's init/tick in order.
 */
#ifndef PDU_APP_H
#define PDU_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise all PDU runtime subsystems.
 *         Called once from main() USER CODE BEGIN 2.
 */
void PDU_App_Init(void);

/**
 * @brief  Single superloop iteration for the PDU runtime.
 *         Called repeatedly from main() while(1) loop.
 */
void PDU_App_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* PDU_APP_H */
