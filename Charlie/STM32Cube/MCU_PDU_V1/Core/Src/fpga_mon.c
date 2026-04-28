/*
 * fpga_mon.c — FPGA I2C monitor, PDU runtime build
 *
 * Driver pattern extracted from pdu_selftest_cli_v3.c:
 *   - Block read registers 0x00–0x06 in a single I2C transfer
 *   - Separate single-byte read for version register 0x7F
 *   - 7-bit address 0x55, left-shifted to 8-bit for HAL
 *   - Timeout 10 ms per transaction
 */
#include "fpga_mon.h"
#include "main.h"

#include <string.h>

/* -----------------------------------------------------------------------
 * Private constants
 * ----------------------------------------------------------------------- */
#define FPGA_ADDR_8BIT        (0x55U << 1)
#define FPGA_I2C_TIMEOUT_MS   10U
#define FPGA_MON_INTERVAL_MS  200U

/* Number of contiguous registers starting at 0x00 */
#define FPGA_BLOCK_LEN        7U   /* 0x00–0x06 inclusive */
#define FPGA_REG_VERSION      0x7FU

/* STATUS0 bit positions */
#define S0_FAULT_LATCH_BIT    0U
#define S0_PCHG_LATCH_BIT     1U
#define S0_MOTOR_EN_BIT       2U
#define S0_COMPUTE_EN_BIT     3U
#define S0_OVUV_OK_BIT        6U
#define S0_ARM_PERMIT_BIT     7U

/* INPUTS bit positions */
#define IN_ESTOP_OK_BIT       2U
#define IN_MCU_CMD_ARM_BIT    3U
#define IN_REMOTE_ARM_BIT     1U

/* -----------------------------------------------------------------------
 * Peripheral reference (owned by CubeMX, resolved at link time)
 * ----------------------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c4;

/* -----------------------------------------------------------------------
 * Module state
 * ----------------------------------------------------------------------- */
static fpga_snapshot_t g_snap;
static uint32_t        g_next_ms;

/* -----------------------------------------------------------------------
 * Private helpers
 * ----------------------------------------------------------------------- */

/**
 * Write register address then read len bytes using a combined I2C transaction
 * (repeated-START between write and read, no STOP).  This matches how the
 * selftest uses HAL_I2C_Mem_Read and is required for the MachXO2 I2C slave to
 * return data from the correct register — it does not retain the register
 * pointer across a STOP condition.
 */
static bool fpga_read_regs(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    return HAL_I2C_Mem_Read(&hi2c4, FPGA_ADDR_8BIT,
                            start_reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len,
                            FPGA_I2C_TIMEOUT_MS) == HAL_OK;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void fpga_mon_init(void)
{
    memset(&g_snap, 0, sizeof(g_snap));
    g_next_ms = 0U;
}

void fpga_mon_tick(uint32_t now_ms)
{
    if (now_ms < g_next_ms) {
        return;
    }
    g_next_ms = now_ms + FPGA_MON_INTERVAL_MS;

    uint8_t block[FPGA_BLOCK_LEN];
    uint8_t ver = 0U;

    if (!fpga_read_regs(0x00U, block, FPGA_BLOCK_LEN)) {
        return;
    }

    /* Version is non-contiguous — separate Mem_Read to 0x7F */
    if (!fpga_read_regs(FPGA_REG_VERSION, &ver, 1U)) {
        return;
    }

    /* Commit to snapshot atomically from the superloop's perspective */
    g_snap.status0       = block[0];
    g_snap.fault_code    = block[1];
    g_snap.state_code    = block[2];
    g_snap.actions       = block[3];
    g_snap.inputs        = block[4];
    g_snap.pchg_timer_ms = (uint16_t)(((uint16_t)block[5] << 8) | block[6]);
    g_snap.version       = ver;
    g_snap.last_read_ms  = now_ms;

    /* Decode STATUS0 flags */
    g_snap.fault_latch    = (block[0] >> S0_FAULT_LATCH_BIT) & 1U;
    g_snap.precharge_latch = (block[0] >> S0_PCHG_LATCH_BIT) & 1U;
    g_snap.motor_en       = (block[0] >> S0_MOTOR_EN_BIT)    & 1U;
    g_snap.compute_en     = (block[0] >> S0_COMPUTE_EN_BIT)  & 1U;
    g_snap.ovuv_ok        = (block[0] >> S0_OVUV_OK_BIT)     & 1U;
    g_snap.arm_permit     = (block[0] >> S0_ARM_PERMIT_BIT)  & 1U;

    /* Decode INPUTS flags */
    g_snap.estop_ok         = (block[4] >> IN_ESTOP_OK_BIT)     & 1U;
    g_snap.mcu_cmd_arm_seen = (block[4] >> IN_MCU_CMD_ARM_BIT)  & 1U;
    g_snap.remote_arm_latch = (block[4] >> IN_REMOTE_ARM_BIT)   & 1U;

    g_snap.valid = true;
}

const fpga_snapshot_t *fpga_mon_get(void)
{
    return &g_snap;
}
