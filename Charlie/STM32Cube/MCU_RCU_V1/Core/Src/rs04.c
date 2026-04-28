/*
 * rs04.c — RobStride RS04 CAN protocol codec
 *
 * Implements the private protocol from RS04 User Manual v1.0, Section 4.
 * All functions are pure (no I/O, no HAL, no global state).
 *
 * 29-bit extended ID construction:
 *   ext_id = (comm_type << 24) | (data16 << 8) | motor_id
 *
 * float_to_uint: maps x in [min, max] → [0, 2^bits - 1], big-endian in data
 */
#include "rs04.h"

#include <string.h>

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Encode float in [min, max] to unsigned integer of given bit width. */
static uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span    = x_max - x_min;
    float clamped = clampf(x, x_min, x_max);
    uint32_t max_raw = (1U << bits) - 1U;
    return (uint32_t)((clamped - x_min) * (float)max_raw / span + 0.5f);
}

/* Decode unsigned integer to float in [min, max]. */
static float uint_to_float(uint32_t raw, float x_min, float x_max, uint8_t bits)
{
    uint32_t max_raw = (1U << bits) - 1U;
    return x_min + ((float)raw / (float)max_raw) * (x_max - x_min);
}

/* Build a 29-bit extended CAN ID from its three fields. */
static uint32_t make_id(uint8_t comm_type, uint16_t data16, uint8_t motor_id)
{
    return ((uint32_t)comm_type << 24)
         | ((uint32_t)data16    << 8)
         | (uint32_t)motor_id;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void rs04_encode_cmd(uint8_t motor_id, const rs04_cmd_t *cmd,
                     uint32_t *out_ext_id, uint8_t out_data[8])
{
    /* Torque feedforward packs into CAN ID bits [23:8] (Type 1 spec) */
    uint16_t trq_id = (uint16_t)float_to_uint(cmd->torque_nm,
                                               -RS04_TRQ_MAX_NM,
                                                RS04_TRQ_MAX_NM, 16U);
    *out_ext_id = make_id(0x01U, trq_id, motor_id);

    /* Position, velocity, Kp, Kd in data bytes — big-endian uint16 each */
    uint32_t p  = float_to_uint(cmd->pos_rad,  -RS04_POS_MAX_RAD,  RS04_POS_MAX_RAD,  16U);
    uint32_t v  = float_to_uint(cmd->vel_rads, -RS04_VEL_MAX_RADS, RS04_VEL_MAX_RADS, 16U);
    uint32_t kp = float_to_uint(cmd->kp,        0.0f, RS04_KP_MAX, 16U);
    uint32_t kd = float_to_uint(cmd->kd,        0.0f, RS04_KD_MAX, 16U);

    out_data[0] = (uint8_t)(p  >> 8);  out_data[1] = (uint8_t)(p  & 0xFFU);
    out_data[2] = (uint8_t)(v  >> 8);  out_data[3] = (uint8_t)(v  & 0xFFU);
    out_data[4] = (uint8_t)(kp >> 8);  out_data[5] = (uint8_t)(kp & 0xFFU);
    out_data[6] = (uint8_t)(kd >> 8);  out_data[7] = (uint8_t)(kd & 0xFFU);
}

bool rs04_decode_feedback(uint32_t ext_id, const uint8_t data[8],
                          rs04_feedback_t *fb)
{
    uint8_t comm_type = (uint8_t)((ext_id >> 24) & 0x1FU);
    if (comm_type != 0x02U) return false;

    uint8_t motor_id    = (uint8_t)((ext_id >> 8)  & 0xFFU);
    uint8_t host_id     = (uint8_t)(ext_id          & 0xFFU);
    uint8_t fault_bits  = (uint8_t)((ext_id >> 16)  & 0x3FU);
    uint8_t mode_status = (uint8_t)((ext_id >> 22)  & 0x03U);

    if (host_id != (uint8_t)RS04_HOST_ID) return false;
    if (motor_id == 0U)                   return false;

    uint32_t p_raw   = ((uint32_t)data[0] << 8) | data[1];
    uint32_t v_raw   = ((uint32_t)data[2] << 8) | data[3];
    uint32_t trq_raw = ((uint32_t)data[4] << 8) | data[5];
    int16_t  temp_raw = (int16_t)(((uint16_t)data[6] << 8) | data[7]);

    fb->motor_id    = motor_id;
    fb->pos_rad     = uint_to_float(p_raw,   -RS04_POS_MAX_RAD,  RS04_POS_MAX_RAD,  16U);
    fb->vel_rads    = uint_to_float(v_raw,   -RS04_VEL_MAX_RADS, RS04_VEL_MAX_RADS, 16U);
    fb->torque_nm   = uint_to_float(trq_raw, -RS04_TRQ_MAX_NM,   RS04_TRQ_MAX_NM,   16U);
    fb->temp_raw    = temp_raw;
    fb->fault_bits  = fault_bits;
    fb->mode_status = mode_status;

    return true;
}

void rs04_encode_enable(uint8_t motor_id,
                        uint32_t *out_ext_id, uint8_t out_data[8])
{
    /* Type 3: host_id in ID[23:8], motor_id in ID[7:0], data all zeros */
    *out_ext_id = make_id(0x03U, (uint16_t)RS04_HOST_ID, motor_id);
    memset(out_data, 0, 8U);
}

void rs04_encode_stop(uint8_t motor_id, bool clear_fault,
                      uint32_t *out_ext_id, uint8_t out_data[8])
{
    /* Type 4: same ID layout as Type 3 */
    *out_ext_id = make_id(0x04U, (uint16_t)RS04_HOST_ID, motor_id);
    memset(out_data, 0, 8U);
    if (clear_fault) {
        out_data[0] = 0x01U;
    }
}

void rs04_encode_param_write(uint8_t motor_id, uint16_t param_id, float value,
                             uint32_t *out_ext_id, uint8_t out_data[8])
{
    /* Type 18 (0x12): host_id in data16 field, motor_id in ID[7:0].      */
    /* Manual section 4.1.7 + code sample 4.4.4 confirm comm_type = 0x12. */
    *out_ext_id = make_id(0x12U, (uint16_t)RS04_HOST_ID, motor_id);
    memset(out_data, 0, 8U);
    /* Byte 0:1 = param_id, little-endian */
    memcpy(&out_data[0], &param_id, sizeof(param_id));
    /* Byte 2:3 = 0x0000 (already zeroed) */
    /* Byte 4:7 = float32, little-endian */
    memcpy(&out_data[4], &value, sizeof(value));
}

void rs04_encode_set_zero(uint8_t motor_id,
                          uint32_t *out_ext_id, uint8_t out_data[8])
{
    /* Type 6: same ID layout as Type 3/4.  Byte[0]=1 per manual section 4.1.5. */
    *out_ext_id = make_id(0x06U, (uint16_t)RS04_HOST_ID, motor_id);
    memset(out_data, 0, 8U);
    out_data[0] = 0x01U;
}
