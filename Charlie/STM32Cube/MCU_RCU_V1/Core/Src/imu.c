/*
 * imu.c — Dual LSM6DSOX IMU driver, RCU runtime build
 *
 * SPI protocol (mode 0, CPOL=0 CPHA=0, MSB first):
 *   Read:  CS low, send (0x80 | reg), send dummy bytes, CS high
 *   Write: CS low, send reg, send data, CS high
 *
 * CubeMX generates SPI3/SPI4 with DataSize=4-bit; spi_reinit_8bit()
 * corrects this to 8-bit before first use.
 *
 * Register access uses HAL_SPI_TransmitReceive for simplicity.
 * CS is manually asserted/deasserted as GPIO.
 */
#include "imu.h"
#include "main.h"

#include <string.h>

/* -----------------------------------------------------------------------
 * LSM6DSOX register addresses
 * ----------------------------------------------------------------------- */
#define REG_WHO_AM_I     0x0FU
#define REG_INT1_CTRL    0x0DU
#define REG_CTRL1_XL     0x10U   /* accelerometer control */
#define REG_CTRL2_G      0x11U   /* gyroscope control */
#define REG_CTRL3_C      0x12U   /* device control */
#define REG_CTRL4_C      0x13U
#define REG_OUT_TEMP_L   0x20U   /* temperature + gyro + accel burst start */

#define LSM6DSOX_WHOAMI  0x6CU

/* CTRL1_XL: ODR 416 Hz (0x6_), full-scale ±4g (0x_8) */
#define CTRL1_XL_VAL     0x68U   /* ODR=0110 (416 Hz), FS=10 (±4g) */
/* CTRL2_G:  ODR 416 Hz (0x6_), full-scale ±500 dps (0x_4) */
#define CTRL2_G_VAL      0x64U   /* ODR=0110 (416 Hz), FS=100 (±500 dps) */
/* CTRL3_C: BDU=1, IF_INC=1 */
#define CTRL3_C_VAL      0x44U
/* INT1_CTRL: DRDY_XL (bit1) and DRDY_G (bit0) */
#define INT1_CTRL_VAL    0x03U

#define SPI_TIMEOUT_MS   5U
#define DATA_BURST_LEN   14U     /* temp(2) + gyro(6) + accel(6) */

/* -----------------------------------------------------------------------
 * Per-IMU descriptor
 * ----------------------------------------------------------------------- */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint16_t           drdy_pin;   /* to match in imu_on_drdy() */
    volatile bool      drdy_flag;
    bool               init_ok;    /* true if init_one succeeded */
    uint32_t           last_poll_ms; /* time of last successful read attempt */
    imu_sample_t       sample;
} imu_dev_t;

/* -----------------------------------------------------------------------
 * Peripheral references (CubeMX-owned)
 * ----------------------------------------------------------------------- */
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi3;

/* -----------------------------------------------------------------------
 * Device table
 * ----------------------------------------------------------------------- */
static imu_dev_t g_dev[IMU_COUNT] = {
    /* IMU0: SPI4, CS=PC13, INT1=PE3 */
    {
        .hspi     = &hspi4,
        .cs_port  = IMU0_NCS_GPIO_Port,
        .cs_pin   = IMU0_NCS_Pin,
        .drdy_pin = IMU0_INT1_Pin,
    },
    /* IMU1: SPI3, CS=PA15, INT1=PA9 */
    {
        .hspi     = &hspi3,
        .cs_port  = IMU1_NCS_GPIO_Port,
        .cs_pin   = IMU1_NCS_Pin,
        .drdy_pin = IMU1_INT1_Pin,
    },
};

/* -----------------------------------------------------------------------
 * SPI reconfiguration
 * CubeMX initialises SPI3/SPI4 with DataSize=4-bit; force 8-bit before use.
 * ----------------------------------------------------------------------- */
static bool spi_reinit_8bit(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_DeInit(hspi);
    hspi->Init.DataSize                = SPI_DATASIZE_8BIT;
    hspi->Init.Direction               = SPI_DIRECTION_2LINES;
    hspi->Init.Mode                    = SPI_MODE_MASTER;
    hspi->Init.CLKPolarity             = SPI_POLARITY_LOW;
    hspi->Init.CLKPhase                = SPI_PHASE_1EDGE;
    hspi->Init.NSS                     = SPI_NSS_SOFT;
    hspi->Init.BaudRatePrescaler       = SPI_BAUDRATEPRESCALER_16;
    hspi->Init.FirstBit                = SPI_FIRSTBIT_MSB;
    hspi->Init.TIMode                  = SPI_TIMODE_DISABLE;
    hspi->Init.CRCCalculation          = SPI_CRCCALCULATION_DISABLE;
    hspi->Init.NSSPMode                = SPI_NSS_PULSE_DISABLE;
    hspi->Init.NSSPolarity             = SPI_NSS_POLARITY_LOW;
    hspi->Init.FifoThreshold           = SPI_FIFO_THRESHOLD_01DATA;
    hspi->Init.MasterReceiverAutoSusp  = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi->Init.MasterKeepIOState       = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi->Init.IOSwap                  = SPI_IO_SWAP_DISABLE;
    return HAL_SPI_Init(hspi) == HAL_OK;
}

/* -----------------------------------------------------------------------
 * Low-level SPI helpers
 * ----------------------------------------------------------------------- */
static inline void cs_low(imu_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}
static inline void cs_high(imu_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static bool spi_write_reg(imu_dev_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg & 0x7FU, value };  /* write: MSB clear */
    uint8_t rx[2];
    cs_low(dev);
    HAL_StatusTypeDef r = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2U, SPI_TIMEOUT_MS);
    cs_high(dev);
    return r == HAL_OK;
}

static bool spi_read_reg(imu_dev_t *dev, uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { 0x80U | reg, 0x00U };
    uint8_t rx[2];
    cs_low(dev);
    HAL_StatusTypeDef r = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2U, SPI_TIMEOUT_MS);
    cs_high(dev);
    *value = rx[1];
    return r == HAL_OK;
}

/* Burst read: send read address then receive 'len' data bytes.
 * Requires IF_INC=1 (set in CTRL3_C). */
static bool spi_read_burst(imu_dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t addr_byte = 0x80U | reg;
    uint8_t dummy[DATA_BURST_LEN + 1];
    uint8_t rxbuf[DATA_BURST_LEN + 1];

    if (len > DATA_BURST_LEN) return false;

    memset(dummy, 0U, len + 1U);
    dummy[0] = addr_byte;

    cs_low(dev);
    HAL_StatusTypeDef r = HAL_SPI_TransmitReceive(dev->hspi, dummy, rxbuf,
                                                   (uint16_t)(len + 1U),
                                                   SPI_TIMEOUT_MS);
    cs_high(dev);
    if (r != HAL_OK) return false;
    memcpy(buf, &rxbuf[1], len);
    return true;
}

/* -----------------------------------------------------------------------
 * Per-IMU initialisation
 * ----------------------------------------------------------------------- */
static bool init_one(imu_dev_t *dev)
{
    /* CubeMX sets DataSize=4-bit; reconfigure to 8-bit for LSM6DSOX */
    if (!spi_reinit_8bit(dev->hspi)) return false;

    cs_high(dev);
    HAL_Delay(1U);

    uint8_t whoami = 0U;
    if (!spi_read_reg(dev, REG_WHO_AM_I, &whoami)) return false;
    if (whoami != LSM6DSOX_WHOAMI)                  return false;

    /* Software reset then wait for completion */
    if (!spi_write_reg(dev, REG_CTRL3_C, 0x01U)) return false;
    HAL_Delay(2U);

    if (!spi_write_reg(dev, REG_CTRL3_C, CTRL3_C_VAL))  return false;
    if (!spi_write_reg(dev, REG_CTRL2_G, CTRL2_G_VAL))  return false;
    if (!spi_write_reg(dev, REG_CTRL1_XL, CTRL1_XL_VAL)) return false;

    /* Enable DRDY on INT1 for both gyro and accel */
    if (!spi_write_reg(dev, REG_INT1_CTRL, INT1_CTRL_VAL)) return false;

    return true;
}

/* -----------------------------------------------------------------------
 * Read a full sample from one device
 * ----------------------------------------------------------------------- */
static void read_sample(imu_dev_t *dev, uint32_t now_ms)
{
    uint8_t raw[DATA_BURST_LEN];
    if (!spi_read_burst(dev, REG_OUT_TEMP_L, raw, DATA_BURST_LEN)) {
        return;
    }

    /* raw[0:1]  = temperature
     * raw[2:7]  = gyro x,y,z (little-endian int16 each)
     * raw[8:13] = accel x,y,z (little-endian int16 each) */
    dev->sample.temp_raw = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);

    dev->sample.gyro[0]  = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    dev->sample.gyro[1]  = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
    dev->sample.gyro[2]  = (int16_t)(((uint16_t)raw[7] << 8) | raw[6]);

    dev->sample.accel[0] = (int16_t)(((uint16_t)raw[9]  << 8) | raw[8]);
    dev->sample.accel[1] = (int16_t)(((uint16_t)raw[11] << 8) | raw[10]);
    dev->sample.accel[2] = (int16_t)(((uint16_t)raw[13] << 8) | raw[12]);

    dev->sample.timestamp_ms = now_ms;
    dev->sample.valid        = true;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

bool imu_init(void)
{
    bool ok = true;
    for (uint8_t i = 0U; i < IMU_COUNT; ++i) {
        g_dev[i].drdy_flag     = false;
        g_dev[i].sample.valid  = false;
        g_dev[i].last_poll_ms  = 0U;
        g_dev[i].init_ok       = init_one(&g_dev[i]);
        ok = g_dev[i].init_ok && ok;
    }
    return ok;
}

void imu_tick(uint32_t now_ms)
{
    for (uint8_t i = 0U; i < IMU_COUNT; ++i) {
        if (!g_dev[i].init_ok) continue;

        bool do_read = g_dev[i].drdy_flag;

        /* Polling fallback: if DRDY has not fired for >200 ms, read anyway.
         * This recovers from a missed EXTI edge (e.g. after reflash while
         * the IMU was already outputting data). */
        if (!do_read && ((now_ms - g_dev[i].last_poll_ms) >= 200U)) {
            do_read = true;
        }

        if (do_read) {
            g_dev[i].drdy_flag    = false;
            g_dev[i].last_poll_ms = now_ms;
            read_sample(&g_dev[i], now_ms);
        }
    }
}

const imu_sample_t *imu_get_sample(uint8_t idx)
{
    if (idx >= IMU_COUNT) return NULL;
    return &g_dev[idx].sample;
}

void imu_on_drdy(uint16_t gpio_pin)
{
    for (uint8_t i = 0U; i < IMU_COUNT; ++i) {
        if (g_dev[i].drdy_pin == gpio_pin) {
            g_dev[i].drdy_flag = true;
        }
    }
}
