/*
 * ssd_energy.h — SSD RS485 energy meter driver for PDU runtime build
 *
 * Queries the SSD energy meter over USART3/RS485 at 19200 baud using
 * its ASCII protocol.  Polling rate ~5 Hz (200 ms).
 *
 * Commands used (address 1, ASCII):
 *   :1GA\r  — current  (prefix 'A', value in mA)
 *   :1GV\r  — voltage  (prefix 'V', value in mV)
 *   :1GT\r  — temp     (prefix 'T', value in 0.1 °C)
 *
 * RS485 direction control: RS485_DE_GPIO_Port / RS485_DE_Pin (active high TX).
 */
#ifndef SSD_ENERGY_H
#define SSD_ENERGY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Snapshot
 * ----------------------------------------------------------------------- */
typedef struct {
    bool     valid;
    uint32_t last_ok_ms;
    float    current_a;    /* [A] */
    float    voltage_v;    /* [V] */
    float    temp_c;       /* [°C] */
} ssd_snapshot_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise the SSD energy module.  Configures UART to 19200-8N1.
 */
void ssd_energy_init(void);

/**
 * @brief  Non-blocking poll.  Call every superloop iteration.
 *         Performs a full I/V/T query at SSD_ENERGY_INTERVAL_MS.
 *         This call is blocking for the duration of 3 × UART round-trips
 *         (~150 ms total) but only fires at the poll interval.
 * @param  now_ms  Current HAL_GetTick() value.
 */
void ssd_energy_tick(uint32_t now_ms);

/**
 * @brief  Return pointer to the latest SSD snapshot.
 */
const ssd_snapshot_t *ssd_energy_get(void);

#ifdef __cplusplus
}
#endif

#endif /* SSD_ENERGY_H */
