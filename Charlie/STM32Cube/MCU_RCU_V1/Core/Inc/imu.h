/*
 * imu.h — Dual LSM6DSOX IMU driver for RCU runtime build
 *
 * IMU0: SPI4 (PE2/PE5/PE6), CS=PC13, INT1=PE3 (EXTI3)
 * IMU1: SPI3 (PC10/PC11/PC12), CS=PA15, INT1=PA9 (EXTI9_5)
 *
 * Operation:
 *   - Both IMUs configured at 416 Hz ODR, ±4g, ±500 dps
 *   - DRDY fires on INT1 when new accel+gyro sample available
 *   - ISR sets a flag; imu_tick() reads the 12-byte data burst
 *   - Latest sample is available via imu_get_sample()
 *
 * Coordinate convention: raw int16 values in SENSOR frame (LSB first, little-endian).
 * Scale: accel 0.122 mg/LSB at ±4g; gyro 17.5 mdps/LSB at ±500 dps.
 *
 * Physical mounting orientation is defined per-IMU in telem_pack.c (IMU0_MOUNT / IMU1_MOUNT).
 * Remap to ROS2 body frame (X=fwd, Y=left, Z=up) is applied in telem_pack_imu_fast().
 */
#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define IMU_COUNT  2U

/* -----------------------------------------------------------------------
 * Raw sample (as read from device, before any unit conversion)
 * ----------------------------------------------------------------------- */
typedef struct {
    bool     valid;
    uint32_t timestamp_ms;
    int16_t  accel[3];   /* [x, y, z] LSB, 0.122 mg/LSB at ±4g */
    int16_t  gyro[3];    /* [x, y, z] LSB, 17.5 mdps/LSB at ±500 dps */
    int16_t  temp_raw;   /* °C = temp_raw/256 + 25 */
} imu_sample_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise both IMUs over SPI.  Configures registers and enables
 *         DRDY interrupt on INT1 of each device.
 *         Returns false if WHO_AM_I check fails for either IMU.
 */
bool imu_init(void);

/**
 * @brief  Non-blocking poll.  Call every superloop iteration.
 *         Reads data from any IMU that has a pending DRDY flag.
 * @param  now_ms  Current HAL_GetTick().
 */
void imu_tick(uint32_t now_ms);

/**
 * @brief  Return pointer to the latest sample for the given IMU index.
 * @param  idx  0 = IMU0, 1 = IMU1.
 */
const imu_sample_t *imu_get_sample(uint8_t idx);

/**
 * @brief  Called from HAL_GPIO_EXTI_Callback (in stm32h7xx_it.c).
 *         Sets the DRDY pending flag for the appropriate IMU.
 * @param  gpio_pin  The pin that triggered (IMU0_INT1_Pin or IMU1_INT1_Pin).
 */
void imu_on_drdy(uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
