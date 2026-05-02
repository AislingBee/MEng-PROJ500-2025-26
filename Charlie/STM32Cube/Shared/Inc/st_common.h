/*
 * st_common.h
 *
 * Shared types, protocol defines, and selector namespace for PDU/RCU
 * self-test framework.  Both boards include this header.
 */

#ifndef ST_COMMON_H
#define ST_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <ctype.h>
#include <string.h>

/* ======================================================================
 *  Test status / runner types
 * ====================================================================== */

typedef enum {
    STS_NOT_RUN = 0,
    STS_RUNNING,
    STS_PASS,
    STS_FAIL,
    STS_WARN,
    STS_SKIPPED
} st_status_t;

typedef enum {
    RUN_IDLE = 0,
    RUN_ALL,
    RUN_ONE,
    RUN_WAIT_LOOP
} st_runner_mode_t;

typedef struct {
    const char *name;
    uint32_t    mask;
    void       (*reset)(void);
    void       (*start)(void);
    st_status_t (*poll)(uint32_t now_ms, char *detail, size_t detail_len);
    st_status_t status;
    char        detail[120];
} st_test_t;

/* ======================================================================
 *  MCAN protocol defines
 * ====================================================================== */

#define ST_MCAN_PROTO_VER            1U

/* Message types */
#define ST_MCAN_TYPE_PING_REQ        0x01U
#define ST_MCAN_TYPE_PING_RESP       0x02U
#define ST_MCAN_TYPE_STATUS_REQ      0x03U
#define ST_MCAN_TYPE_STATUS_RESP     0x04U
#define ST_MCAN_TYPE_LED_REQ         0x05U
#define ST_MCAN_TYPE_BEEP_REQ        0x06U
#define ST_MCAN_TYPE_ACK             0x07U
#define ST_MCAN_TYPE_CLEAR_STATS_REQ 0x08U
#define ST_MCAN_TYPE_SELFTEST_REQ    0x09U
#define ST_MCAN_TYPE_SELFTEST_ACK    0x0AU
#define ST_MCAN_TYPE_SELFTEST_RESULT 0x0BU
#define ST_MCAN_TYPE_SELFTEST_DONE   0x0CU

/* Timing defaults */
#define ST_MCAN_DEFAULT_HB_MS        500U
#define ST_MCAN_DEFAULT_PING_MS      500U
#define ST_MCAN_PING_TIMEOUT_MS      300U

/* ======================================================================
 *  Selector namespace (global, both boards)
 * ====================================================================== */

#define ST_MCAN_SEL_ALL              0U
#define ST_MCAN_SEL_GPIO             1U
#define ST_MCAN_SEL_BUZZER           2U
#define ST_MCAN_SEL_MCAN             3U
#define ST_MCAN_SEL_CAN              4U
#define ST_MCAN_SEL_SSD              5U
#define ST_MCAN_SEL_FPGA             6U
#define ST_MCAN_SEL_ADC              7U
#define ST_MCAN_SEL_THERM            8U
#define ST_MCAN_SEL_PDUFAULT         9U
#define ST_MCAN_SEL_CAN_PDU          10U
#define ST_MCAN_SEL_CAN_LEFT         11U
#define ST_MCAN_SEL_CAN_RIGHT        12U
#define ST_MCAN_SEL_ETH              13U
#define ST_MCAN_SEL_IMU0             14U
#define ST_MCAN_SEL_IMU1             15U
#define ST_MCAN_SEL_ESP              16U
#define ST_MCAN_SEL_EXP              17U
#define ST_MCAN_SEL_COUNT            18U

/* ======================================================================
 *  MCAN action types
 * ====================================================================== */

typedef enum {
    ST_MCAN_ACT_NONE = 0,
    ST_MCAN_ACT_LED,
    ST_MCAN_ACT_BEEP
} st_mcan_action_t;

/* ======================================================================
 *  MCAN runtime state  (protocol / comms only)
 * ====================================================================== */

typedef struct {
    /* bus state */
    bool        online;
    bool        monitor;

    /* heartbeat */
    bool        hb_enabled;
    uint32_t    hb_period_ms;
    uint32_t    last_hb_tx_ms;

    /* peer presence */
    uint32_t    last_peer_seen_ms;
    bool        peer_online;

    /* ping campaign */
    bool        ping_active;
    bool        ping_both;
    bool        ping_report_each;
    uint32_t    ping_period_ms;
    uint32_t    ping_target_count;
    uint32_t    ping_completed_count;
    uint32_t    ping_sent;
    uint32_t    ping_resp;
    uint32_t    ping_req_rx;
    uint32_t    ping_timeout;

    /* error counters */
    uint32_t    bad_ver;
    uint32_t    bad_type;
    uint32_t    bad_dlc;
    uint32_t    bad_seq;
    uint32_t    dup_resp;
    uint32_t    tx_queue_fail;
    uint32_t    bus_off_count;

    /* bus timing */
    uint32_t    bus_online_ms;

    /* frame counters */
    uint32_t    tx_frames;
    uint32_t    rx_frames;
    uint32_t    tx_hb;
    uint32_t    rx_hb;
    uint32_t    tx_req;
    uint32_t    rx_req;
    uint32_t    tx_resp;
    uint32_t    rx_resp;
    uint32_t    ack_rx;

    /* last-received bookkeeping */
    uint32_t    status_resp_rx;
    uint8_t     last_status_seq;
    uint16_t    last_status_word;
    uint8_t     last_ack_seq;
    uint16_t    last_ack_type;
    uint32_t    last_ack_ms;

    /* remote selftest tracking */
    bool        remote_selftest_active;
    uint8_t     remote_selftest_req_seq;
    uint16_t    remote_selftest_root_selector;

    /* peer selftest summary (from last SELFTEST_DONE frame received) */
    uint16_t    peer_selftest_pass;
    uint16_t    peer_selftest_warn;
    uint16_t    peer_selftest_fail;
    bool        peer_selftest_valid;

    /* sequence numbering */
    uint8_t     next_seq;

    /* ping RTT state */
    bool        ping_waiting;
    uint8_t     awaiting_seq;
    uint32_t    ping_tx_ms;
    uint32_t    ping_last_due_ms;
    uint32_t    rtt_last_ms;
    uint32_t    rtt_min_ms;
    uint32_t    rtt_max_ms;
    uint64_t    rtt_sum_ms;
    uint32_t    rtt_count;

    /* pending action (LED blink / beep from remote request) */
    st_mcan_action_t action_type;
    uint16_t    action_arg0;
    uint16_t    action_arg1;
    uint16_t    action_remaining;
    bool        action_state;
    uint32_t    action_next_ms;
} st_mcan_rt_t;

/* ======================================================================
 *  PDU status word bit positions
 *  Transmitted in HEARTBEAT + STATUS_RESP from PDU.
 *  Decoded by RCU when displaying peer status.
 * ====================================================================== */

#define MCAN_PDU_SW_FAULT_LATCH       (1u<<0)   /* FAULT_LATCH GPIO */
#define MCAN_PDU_SW_FAULT_BTN         (1u<<1)   /* SW_FAULT GPIO */
#define MCAN_PDU_SW_PGOOD_SW          (1u<<2)   /* PGOOD_SW GPIO */
#define MCAN_PDU_SW_PGOOD_24V         (1u<<3)   /* PGOOD_24V GPIO */
#define MCAN_PDU_SW_FPGA_STATE_SHIFT  4u        /* 2-bit field: 0=IDLE,1=PRECHARGE,2=ARMED,3=COMPUTE */
#define MCAN_PDU_SW_FPGA_STATE_MASK   (3u<<4)
#define MCAN_PDU_SW_FPGA_FLTL         (1u<<6)   /* FPGA fault_latch */
#define MCAN_PDU_SW_ARM_PERMIT        (1u<<7)   /* FPGA arm_permit */
#define MCAN_PDU_SW_ESTOP_OK          (1u<<8)   /* FPGA estop_ok */

/* ======================================================================
 *  Inline helpers
 * ====================================================================== */

static inline const char *st_status_str(st_status_t s)
{
    switch (s) {
        case STS_NOT_RUN: return "NOT_RUN";
        case STS_RUNNING: return "RUNNING";
        case STS_PASS:    return "PASS";
        case STS_FAIL:    return "FAIL";
        case STS_WARN:    return "WARN";
        case STS_SKIPPED: return "SKIPPED";
        default:          return "?";
    }
}

static inline bool st_str_eq_nocase(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b))
            return false;
        ++a; ++b;
    }
    return (*a == '\0' && *b == '\0');
}

/* ======================================================================
 *  Selector name/ID mapping  (table-driven, global namespace)
 * ====================================================================== */

typedef struct {
    uint16_t    id;
    const char *name;
} st_mcan_sel_entry_t;

static const st_mcan_sel_entry_t st_mcan_sel_table[ST_MCAN_SEL_COUNT] = {
    { ST_MCAN_SEL_ALL,       "all"       },
    { ST_MCAN_SEL_GPIO,      "gpio"      },
    { ST_MCAN_SEL_BUZZER,    "buzzer"    },
    { ST_MCAN_SEL_MCAN,      "mcan"      },
    { ST_MCAN_SEL_CAN,       "can"       },
    { ST_MCAN_SEL_SSD,       "ssd"       },
    { ST_MCAN_SEL_FPGA,      "fpga"      },
    { ST_MCAN_SEL_ADC,       "adc"       },
    { ST_MCAN_SEL_THERM,     "therm"     },
    { ST_MCAN_SEL_PDUFAULT,  "pdufault"  },
    { ST_MCAN_SEL_CAN_PDU,   "can_pdu"   },
    { ST_MCAN_SEL_CAN_LEFT,  "can_left"  },
    { ST_MCAN_SEL_CAN_RIGHT, "can_right" },
    { ST_MCAN_SEL_ETH,       "eth"       },
    { ST_MCAN_SEL_IMU0,      "imu0"      },
    { ST_MCAN_SEL_IMU1,      "imu1"      },
    { ST_MCAN_SEL_ESP,       "esp"       },
    { ST_MCAN_SEL_EXP,       "exp"       },
};

static inline const char *st_mcan_selector_name(uint16_t sel)
{
    if (sel < ST_MCAN_SEL_COUNT)
        return st_mcan_sel_table[sel].name;
    return "?";
}

static inline int st_mcan_selector_from_name(const char *name)
{
    for (uint16_t i = 0U; i < ST_MCAN_SEL_COUNT; ++i) {
        if (st_str_eq_nocase(name, st_mcan_sel_table[i].name))
            return (int)st_mcan_sel_table[i].id;
    }
    return -1;
}

#ifdef __cplusplus
}
#endif

#endif /* ST_COMMON_H */
