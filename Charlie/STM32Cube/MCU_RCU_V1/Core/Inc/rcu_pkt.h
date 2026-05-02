/*
 * rcu_pkt.h — UDP binary packet definitions, RCU ↔ Thor
 *
 * All multi-byte fields are little-endian.
 *
 * Header (6 bytes):
 *   [0:1]  magic   = 0x5243 ('RC' LE)
 *   [2]    type    = packet type (see RCU_PKT_TYPE_xxx)
 *   [3]    seq     = rolling sequence counter (wraps 0–255)
 *   [4:5]  len     = payload length in bytes (LE uint16)
 *
 * Total frame = 6 + len bytes.
 *
 * Packet types:
 *   0x01  Slow telemetry    (RCU → Thor, 10 Hz)
 *   0x02  Motor feedback    (RCU → Thor, 200 Hz)
 *   0x03  Supervision event (RCU → Thor, on-change)
 *   0x04  Fast IMU          (RCU → Thor, 200 Hz)
 *   0x10  Motor command     (Thor → RCU)
 *   0x11  Motor supervisory (Thor → RCU, enable/ctrl_mode/fault-clear)
 */
#ifndef RCU_PKT_H
#define RCU_PKT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -----------------------------------------------------------------------
 * Magic and types
 * ----------------------------------------------------------------------- */
#define RCU_PKT_MAGIC    0x5243U   /* 'RC' little-endian */
#define RCU_PKT_HDR_SIZE 6U

#define RCU_PKT_TYPE_SLOW_TELEM    0x01U
#define RCU_PKT_TYPE_MOTOR_FB      0x02U
#define RCU_PKT_TYPE_SUPERVISION   0x03U
#define RCU_PKT_TYPE_IMU_FAST      0x04U   /* 200 Hz IMU for control loop */
#define RCU_PKT_TYPE_MOTOR_CMD     0x10U
#define RCU_PKT_TYPE_MOTOR_SUPV    0x11U   /* Thor → RCU: enable/ctrl_mode  */
#define RCU_PKT_TYPE_DEBUG_CMD     0x20U   /* PC → RCU: single byte sub-command */
#define RCU_PKT_TYPE_DEBUG_REPLY   0x21U   /* RCU → PC: rcu_debug_reply_t */

/* Debug sub-command byte (payload[0] of RCU_PKT_TYPE_DEBUG_CMD) */
#define RCU_DBGCMD_PING            0x01U   /* Request status reply immediately */
#define RCU_DBGCMD_BUZZ            0x02U   /* 200 ms buzzer pulse */
#define RCU_DBGCMD_LED_BLINK       0x03U   /* Blink orange LED 3x */
#define RCU_DBGCMD_CAN_LOOPBACK    0x04U   /* Run FDCAN1+FDCAN3 loopback, reply result */
#define RCU_DBGCMD_FORCE_TELEM     0x05U   /* Force immediate slow-telem TX */
#define RCU_DBGCMD_ASSERT_PDU_FAULT  0x07U /* payload[1]: 1=assert 0=deassert */
#define RCU_DBGCMD_SOFT_RESET        0x08U /* payload[1]: 0=RCU only, 1=RCU+PDU */
#define RCU_DBGCMD_SET_TELEM_RATE    0x09U /* payload[1]: Hz (5, 10, or 20) */
#define RCU_DBGCMD_MOTOR_BUS_CTRL    0x0AU /* payload[1]: bit0=L_STB, bit1=R_STB */
#define RCU_DBGCMD_REQUEST_SUPV_DUMP 0x0BU /* no extra payload */
#define RCU_DBGCMD_MOTOR_ENABLE      0x0CU /* payload: bus(u8), motor_id(u8), enable(u8), clr_fault(u8) */
#define RCU_DBGCMD_MOTOR_SET_ZERO    0x0DU /* payload: bus(u8), motor_id(u8) — set current pos as zero */
/* -----------------------------------------------------------------------
 * Header struct (packed)
 * ----------------------------------------------------------------------- */
#pragma pack(push, 1)

typedef struct {
    uint16_t magic;
    uint8_t  type;
    uint8_t  seq;
    uint16_t len;    /* payload length */
} rcu_pkt_hdr_t;

/* -----------------------------------------------------------------------
 * Type 0x01 — Slow telemetry payload (RCU → Thor, 10 Hz)
 *
 * Wire size: 72 bytes (little-endian, packed).
 *
 * PDU — FPGA (8 B):
 *   fpga_status0    uint8
 *   fpga_fault_code uint8
 *   fpga_state_code uint8
 *   fpga_actions    uint8
 *   fpga_inputs     uint8   [FPGA reg 0x04: estop/arm flags]
 *   fpga_version    uint8   [FPGA reg 0x7F]
 *   fpga_pchg_ms    uint16  [precharge timer ms]
 * PDU — external ADC / TLA2528 (16 B):
 *   v_vraw_dv       int16   [10mV units — divide by 100 for V; packed ×100 to fit int16]
 *   v_12v_mv        int16   [mV]
 *   v_24v_mv        int16   [mV]
 *   i_vraw_sw_ma    int16   [mA — current of the switched VRAW output]
 *   i_12v_ma        int16   [mA]
 *   i_24v_ma        int16   [mA]
 *   therm1_dc       int16   [0.1 °C]
 *   therm2_dc       int16   [0.1 °C]
 * PDU — SSD energy meter (8 B):
 *   ssd_i_ma        int16   [mA]
 *   ssd_v_dv        int16   [10mV units — divide by 100 for Volts]
 *   ssd_p_dw        int16   [0.1 W]
 *   ssd_t_dc        int16   [0.1 °C]
 * PDU — local STM32 ADC (12 B):
 *   ladc_therm0_dc  int16   [0.1 °C]
 *   ladc_therm1_dc  int16   [0.1 °C]
 *   ladc_therm2_dc  int16   [0.1 °C]
 *   ladc_vsource_mv int16   [mV]
 *   ladc_vbus_mv    int16   [mV]
 *   ladc_icoil_ma   int16   [mA]
 * IMU0 section (14 B):
 *   accel[3]        int16 × 3
 *   gyro[3]         int16 × 3
 *   temp_raw        int16
 * IMU1 section (14 B): same layout as IMU0
 * ----------------------------------------------------------------------- */
typedef struct {
    /* PDU — FPGA */
    uint8_t  fpga_status0;
    uint8_t  fpga_fault_code;
    uint8_t  fpga_state_code;
    uint8_t  fpga_actions;
    uint8_t  fpga_inputs;         /* FPGA register 0x04 */
    uint8_t  fpga_version;        /* FPGA register 0x7F */
    uint16_t fpga_pchg_ms;        /* precharge timer [ms] */
    /* PDU — external ADC */
    int16_t  v_vraw_dv;           /* V_RAW 10mV units (div by 100 for V) */
    int16_t  v_12v_mv;
    int16_t  v_24v_mv;
    int16_t  i_vraw_sw_ma;        /* switched VRAW output current [mA] */
    int16_t  i_12v_ma;
    int16_t  i_24v_ma;
    int16_t  therm1_dc;           /* ext thermistor 1 [0.1 °C] */
    int16_t  therm2_dc;           /* ext thermistor 2 [0.1 °C] */
    /* PDU — SSD energy meter */
    int16_t  ssd_i_ma;
    int16_t  ssd_v_dv;
    int16_t  ssd_p_dw;            /* power [0.1 W] */
    int16_t  ssd_t_dc;
    /* PDU — local STM32 ADC */
    int16_t  ladc_therm0_dc;      /* board thermistor 0 [0.1 °C] */
    int16_t  ladc_therm1_dc;
    int16_t  ladc_therm2_dc;
    int16_t  ladc_vsource_mv;     /* V_SOURCE [10mV units] ÷100 for V */
    int16_t  ladc_vbus_mv;        /* V_BUS (motor bus) [10mV units] ÷100 for V */
    int16_t  ladc_icoil_ma;       /* I_COIL [mA] */
    /* IMU0 */
    int16_t  imu0_accel[3];
    int16_t  imu0_gyro[3];
    int16_t  imu0_temp;
    /* IMU1 */
    int16_t  imu1_accel[3];
    int16_t  imu1_gyro[3];
    int16_t  imu1_temp;
} rcu_telem_payload_t;

/* -----------------------------------------------------------------------
 * Type 0x02 — Motor feedback payload (RCU → Thor)
 *
 * Fixed-size array of 16 motor feedback slots (8 right + 8 left).
 * Slot layout:
 *   bus       uint8   (0=right, 1=left)
 *   motor_id  uint8
 *   pos_u16   uint16  (encoded float, same scale as RS04 cmd)
 *   vel_u16   uint16
 *   cur_u16   uint16  (current/torque proxy)
 *   error     uint8
 *   _pad      uint8   (alignment)
 * ----------------------------------------------------------------------- */
typedef struct {
    uint8_t  bus;
    uint8_t  motor_id;
    uint16_t pos_u16;
    uint16_t vel_u16;
    uint16_t cur_u16;
    uint8_t  error_code;
    uint8_t  mode_status;  /* bits 23-22 of Type-2 CAN ID: 0=idle, 1=MIT, 2+=other */
} rcu_motor_fb_slot_t;

#define RCU_MOTOR_FB_SLOTS  16U

typedef struct {
    uint8_t            count;   /* number of valid slots */
    uint8_t            _pad[3];
    rcu_motor_fb_slot_t slot[RCU_MOTOR_FB_SLOTS];
} rcu_motor_fb_payload_t;

/* -----------------------------------------------------------------------
 * Type 0x10 — Motor command payload (Thor → RCU)
 *
 * Variable-length array of motor command entries, each:
 *   bus       uint8
 *   motor_id  uint8
 *   pos_u16   uint16
 *   vel_u16   uint16
 *   trq_u16   uint16
 *   kp_u8     uint8
 *   kd_u8     uint8
 * ----------------------------------------------------------------------- */
typedef struct {
    uint8_t  bus;
    uint8_t  motor_id;
    uint16_t pos_u16;
    uint16_t vel_u16;
    uint16_t trq_u16;
    uint16_t kp_u16;   /* 0..65535 → 0..RS04_KP_MAX */
    uint16_t kd_u16;   /* 0..65535 → 0..RS04_KD_MAX */
} rcu_motor_cmd_entry_t;

/* -----------------------------------------------------------------------
 * Type 0x04 — Fast IMU packet (RCU → Thor, 200 Hz)
 *
 * Wire size: 28 bytes.  Excludes temperature to keep the packet small;
 * temperature is available in the 10 Hz slow-telem packet.
 * Scale factors: same as the IMU fields in rcu_telem_payload_t.
 *   accel: 0.122 mg/LSB at ±4g
 *   gyro:  17.5 mdps/LSB at ±500 dps
 * ----------------------------------------------------------------------- */
typedef struct {
    int16_t  imu0_accel[3];  /* IMU0 accelerometer  [x,y,z] raw */
    int16_t  imu0_gyro[3];   /* IMU0 gyroscope      [x,y,z] raw */
    int16_t  imu1_accel[3];  /* IMU1 accelerometer  [x,y,z] raw */
    int16_t  imu1_gyro[3];   /* IMU1 gyroscope      [x,y,z] raw */
    uint32_t tick_ms;        /* HAL_GetTick() at time of packing */
} rcu_imu_fast_t;

/* -----------------------------------------------------------------------
 * Type 0x11 — Motor supervisory packet (Thor → RCU)
 *
 * Wire size: 8 bytes.
 * enable_mask:      bit N = enable motor (N+1).  Bus from MOTOR_BUS_MAP.
 * clear_fault_mask: bit N = clear fault for motor (N+1) before enabling.
 * ctrl_mode:        0 = Type-1 MIT impedance (Phase 2),
 *                   1 = CSP param-write position (Phase 1, default).
 * ----------------------------------------------------------------------- */
typedef struct {
    uint16_t enable_mask;       /* bit N = enable motor_id N+1 */
    uint16_t clear_fault_mask;  /* bit N = clear fault for motor_id N+1 */
    uint8_t  ctrl_mode;         /* 0=MIT Type1, 1=CSP param-write */
    uint8_t  _pad[3];
} rcu_motor_supervisory_t;

/* -----------------------------------------------------------------------
 * Type 0x21 — Debug reply payload (RCU → PC)
 * Sent in response to any RCU_PKT_TYPE_DEBUG_CMD packet.
 * ----------------------------------------------------------------------- */
typedef struct {
    uint32_t uptime_ms;
    uint32_t boot_rsr;          /* RCC->RSR value captured at startup     */
    uint8_t  imu0_valid;        /* 1 = has received at least one sample   */
    uint8_t  imu1_valid;
    uint8_t  pdu_fpga_valid;    /* 1 = 0x520 received at least once       */
    uint8_t  pdu_rails_valid;   /* 1 = 0x521 received                     */
    uint8_t  pdu_ssd_valid;     /* 1 = 0x523 received                     */
    uint8_t  can_loopback;      /* 0=untested, 1=right OK, 2=left OK,     */
                                /* 3=both OK, 0xFF=fail                   */
    uint8_t  _pad[2];
    uint32_t pdu_hb_age_ms;     /* ms since last 0x511; 0xFFFFFFFF=never  */
} rcu_debug_reply_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* RCU_PKT_H */
