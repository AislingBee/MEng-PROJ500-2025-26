/*
 * rs04.h — RobStride RS04 CAN protocol codec (pure, no HAL)
 *
 * Implements the RS04 private protocol (Chapter 4, RS04 User Manual v1.0).
 * Uses 29-bit extended CAN frames at 1 Mbps.
 *
 * 29-bit extended ID field layout (all frames):
 *   bits [28:24]  communication type (comm_type)
 *   bits [23:8]   data area 2 (type-dependent)
 *   bits [7:0]    destination: motor CAN_ID
 *
 * Type 1 — Operation control (host→motor):
 *   ID[28:24] = 0x01
 *   ID[23:8]  = torque feedforward uint16 (float_to_uint, -120~120 Nm)
 *   ID[7:0]   = motor_id
 *   Data[0:1] = position uint16 big-endian, range -4π~4π rad (-12.57~12.57)
 *   Data[2:3] = velocity uint16 big-endian, range -15~15 rad/s
 *   Data[4:5] = Kp       uint16 big-endian, range 0~5000
 *   Data[6:7] = Kd       uint16 big-endian, range 0~100
 *
 * Type 2 — Feedback (motor→host, response to any command):
 *   ID[28:24] = 0x02
 *   ID[23:22] = mode_status (0=reset, 1=cali, 2=run)
 *   ID[21:16] = fault bits
 *   ID[15:8]  = motor CAN_ID
 *   ID[7:0]   = host CAN_ID
 *   Data[0:1] = current angle uint16 big-endian, -4π~4π rad
 *   Data[2:3] = current velocity uint16 big-endian, -15~15 rad/s
 *   Data[4:5] = current torque uint16 big-endian, -120~120 Nm
 *   Data[6:7] = winding temperature (Celsius * 10), big-endian
 *
 * Type 3 — Motor enable (host→motor), data = all zeros
 * Type 4 — Motor stop  (host→motor), data = all zeros (byte[0]=1 to clear fault)
 */
#ifndef RS04_H
#define RS04_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Physical parameter limits (from manual Section 4.4, defines section)
 * ----------------------------------------------------------------------- */
#define RS04_POS_MAX_RAD     12.57f   /* ±4π rad */
#define RS04_VEL_MAX_RADS    15.0f    /* ±15 rad/s */
#define RS04_TRQ_MAX_NM     120.0f    /* ±120 N·m */
#define RS04_KP_MAX        5000.0f    /* 0~5000 */
#define RS04_KD_MAX         100.0f    /* 0~100 */

/* Host (master) CAN ID sent in outgoing frames.  Motors default to 0xFD. */
#define RS04_HOST_ID         0xFDU

/* -----------------------------------------------------------------------
 * Type-18 (comm_type=0x12) parameter indices used for CSP position mode.
 * Source: RS04 User Manual v1.0, Section 4.1.14 (parameter table).
 * ----------------------------------------------------------------------- */
#define RS04_PARAM_RUN_MODE   0x7005U  /* uint8 via float: 0=MIT/op, 5=CSP pos */
#define RS04_PARAM_LOC_REF    0x7016U  /* float [rad]: CSP position target     */
#define RS04_PARAM_LIMIT_SPD  0x7017U  /* float [rad/s]: CSP speed limit       */

/* -----------------------------------------------------------------------
 * Structured command / feedback
 * ----------------------------------------------------------------------- */
typedef struct {
    float pos_rad;     /* desired position         [rad] */
    float vel_rads;    /* desired velocity         [rad/s] */
    float torque_nm;   /* feedforward torque       [N·m] */
    float kp;          /* position gain            [N·m/rad] */
    float kd;          /* velocity damping         [N·m·s/rad] */
} rs04_cmd_t;

typedef struct {
    uint8_t motor_id;
    float   pos_rad;      /* load-end position    [rad] */
    float   vel_rads;     /* load-end velocity    [rad/s] */
    float   torque_nm;    /* current torque       [N·m] */
    int16_t temp_raw;     /* winding temp * 10    [0.1 °C] */
    uint8_t fault_bits;   /* ID bits [21:16] */
    uint8_t mode_status;  /* ID bits [23:22]: 0=reset,1=cali,2=run */
} rs04_feedback_t;

/* -----------------------------------------------------------------------
 * Codec API (pure, no HAL)
 * ----------------------------------------------------------------------- */

/**
 * @brief  Encode a Type-1 control command into a CAN extended ID + 8-byte payload.
 * @param  motor_id   Target motor ID (1–127).
 * @param  cmd        Command parameters.
 * @param  out_ext_id Output: 29-bit extended CAN ID.
 * @param  out_data   Output: 8 data bytes.
 */
void rs04_encode_cmd(uint8_t motor_id, const rs04_cmd_t *cmd,
                     uint32_t *out_ext_id, uint8_t out_data[8]);

/**
 * @brief  Decode a received CAN frame into a feedback structure.
 * @param  ext_id   Received 29-bit extended CAN ID.
 * @param  data     Received 8 data bytes.
 * @param  fb       Output: decoded feedback.
 * @return true if the frame is a valid RS04 feedback frame.
 */
bool rs04_decode_feedback(uint32_t ext_id, const uint8_t data[8],
                          rs04_feedback_t *fb);

/**
 * @brief  Build a Type-3 enable frame (motor start).
 */
void rs04_encode_enable(uint8_t motor_id,
                        uint32_t *out_ext_id, uint8_t out_data[8]);

/**
 * @brief  Build a Type-4 stop frame.
 * @param  clear_fault  true = set byte[0]=1 to clear faults before stopping.
 */
void rs04_encode_stop(uint8_t motor_id, bool clear_fault,
                      uint32_t *out_ext_id, uint8_t out_data[8]);

/**
 * @brief  Build a Type-18 (comm_type=0x12) single-parameter write frame.
 *         Confirmed per RS04 User Manual v1.0, Section 4.1.7 + code sample 4.4.4.
 *         Data layout: [0:1]=param_id LE uint16, [2:3]=0x0000, [4:7]=float32 LE.
 * @param  motor_id  Target motor CAN ID.
 * @param  param_id  Parameter index (e.g. RS04_PARAM_RUN_MODE).
 * @param  value     Float value to write (LE float32 in bytes 4–7).
 */
void rs04_encode_param_write(uint8_t motor_id, uint16_t param_id, float value,
                             uint32_t *out_ext_id, uint8_t out_data[8]);

/**
 * @brief  Build a Type-6 set-mechanical-zero frame.
 *         Motor defines current position as zero.  Byte[0]=1 per manual.
 */
void rs04_encode_set_zero(uint8_t motor_id,
                          uint32_t *out_ext_id, uint8_t out_data[8]);

#ifdef __cplusplus
}
#endif

#endif /* RS04_H */
