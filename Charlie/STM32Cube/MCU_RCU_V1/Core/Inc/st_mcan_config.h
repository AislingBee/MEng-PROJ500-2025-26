/*
 * st_mcan_config.h  —  RCU board configuration for shared MCAN layer
 *
 * This header is included by st_mcan.h to obtain HAL types and board-specific
 * CAN IDs.  Each board provides its own version on its include path.
 */

#ifndef ST_MCAN_CONFIG_H
#define ST_MCAN_CONFIG_H

#include "stm32h7xx_hal.h"

#define ST_MCAN_MY_MSG_ID    0x500U
#define ST_MCAN_PEER_MSG_ID  0x501U
#define ST_MCAN_MY_HB_ID     0x510U
#define ST_MCAN_PEER_HB_ID   0x511U

/* -----------------------------------------------------------------------
 * Runtime telemetry IDs received from PDU (runtime build only)
 * ----------------------------------------------------------------------- */
#define MCAN_ID_FPGA_STATUS  0x520U  /* FPGA state snapshot,  10 Hz, 4B */
#define MCAN_ID_PWR_RAILS    0x521U  /* Voltage rails,        10 Hz, 8B */
#define MCAN_ID_CURR_TEMP    0x522U  /* Currents + therms,    10 Hz, 8B */
#define MCAN_ID_SSD          0x523U  /* SSD energy meter,      5 Hz, 8B */

/* Runtime command IDs sent to PDU (runtime build only) */
#define MCAN_ID_CMD_AUX      0x530U  /* AUX switch command,    1B  */
#define MCAN_ID_CMD_FAULT    0x531U  /* CMD_FAULT request,     1B  */

#endif /* ST_MCAN_CONFIG_H */
