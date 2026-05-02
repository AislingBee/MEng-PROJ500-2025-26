/*
 * telem_pack.c — Telemetry packet assembler
 *
 * Converts live subsystem state into wire-format payloads (rcu_pkt.h types).
 * rs04_encode_cmd / rs04 float_to_uint helpers are duplicated inline here
 * to avoid exposing private codec internals; alternatively motor_bus_send_cmd
 * accepts rs04_cmd_t directly.
 */
#include "telem_pack.h"
#include "mcan_pdu.h"
#include "imu.h"
#include "motor_bus.h"
#include "rs04.h"
#include "main.h"

#include <string.h>

/* -----------------------------------------------------------------------
 * Helpers: float ↔ uint16 in RS04 range (mirrors rs04.c internals)
 * ----------------------------------------------------------------------- */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint16_t f_to_u16(float v, float min, float max)
{
    float norm = (clampf(v, min, max) - min) / (max - min);
    return (uint16_t)(norm * 65535.0f + 0.5f);
}

static float u16_to_f(uint16_t raw, float min, float max)
{
    return min + ((float)raw / 65535.0f) * (max - min);
}

/* -----------------------------------------------------------------------
 * Module state — motor supervisory
 * ----------------------------------------------------------------------- */

/* Current control mode: 0=MIT Type1, 1=CSP param-write */
static uint8_t  g_ctrl_mode    = 0U;
/* Bitmask of currently enabled motors (bit N = motor_id N+1) */
static uint16_t g_enabled_mask = 0U;

/* -----------------------------------------------------------------------
 * Slow telemetry
 * ----------------------------------------------------------------------- */
void telem_pack_slow(rcu_telem_payload_t *out)
{
    memset(out, 0, sizeof(*out));

    const pdu_telem_t *pdu = mcan_pdu_get_telem();

    out->fpga_status0    = pdu->fpga_status0;
    out->fpga_fault_code = pdu->fpga_fault_code;
    out->fpga_state_code = pdu->fpga_state_code;
    out->fpga_actions    = pdu->fpga_actions;
    out->fpga_inputs     = pdu->fpga_inputs;
    out->fpga_version    = pdu->fpga_version;
    out->fpga_pchg_ms    = pdu->fpga_pchg_ms;

    out->v_vraw_dv    = pdu->v_vraw_dv;
    out->v_12v_mv     = pdu->v_12v_mv;
    out->v_24v_mv     = pdu->v_24v_mv;
    out->i_vraw_sw_ma = pdu->i_vraw_sw_ma;
    out->i_12v_ma   = pdu->i_12v_ma;
    out->i_24v_ma   = pdu->i_24v_ma;
    out->therm1_dc  = pdu->therm1_dc;
    out->therm2_dc  = pdu->therm2_dc;

    out->ssd_i_ma   = pdu->ssd_i_ma;
    out->ssd_v_dv   = pdu->ssd_v_dv;
    out->ssd_p_dw   = pdu->ssd_p_dw;
    out->ssd_t_dc   = pdu->ssd_t_dc;

    out->ladc_therm0_dc  = pdu->ladc_therm0_dc;
    out->ladc_therm1_dc  = pdu->ladc_therm1_dc;
    out->ladc_therm2_dc  = pdu->ladc_therm2_dc;
    out->ladc_vsource_mv = pdu->ladc_vsource_mv;
    out->ladc_vbus_mv    = pdu->ladc_vbus_mv;
    out->ladc_icoil_ma   = pdu->ladc_icoil_ma;

    const imu_sample_t *s0 = imu_get_sample(0U);
    if (s0 && s0->valid) {
        out->imu0_accel[0] = s0->accel[0];
        out->imu0_accel[1] = s0->accel[1];
        out->imu0_accel[2] = s0->accel[2];
        out->imu0_gyro[0]  = s0->gyro[0];
        out->imu0_gyro[1]  = s0->gyro[1];
        out->imu0_gyro[2]  = s0->gyro[2];
        out->imu0_temp     = s0->temp_raw;
    }

    const imu_sample_t *s1 = imu_get_sample(1U);
    if (s1 && s1->valid) {
        out->imu1_accel[0] = s1->accel[0];
        out->imu1_accel[1] = s1->accel[1];
        out->imu1_accel[2] = s1->accel[2];
        out->imu1_gyro[0]  = s1->gyro[0];
        out->imu1_gyro[1]  = s1->gyro[1];
        out->imu1_gyro[2]  = s1->gyro[2];
        out->imu1_temp     = s1->temp_raw;
    }
}

/* -----------------------------------------------------------------------
 * Motor feedback
 * ----------------------------------------------------------------------- */
void telem_pack_motor_fb(rcu_motor_fb_payload_t *out)
{
    memset(out, 0, sizeof(*out));
    uint8_t slot = 0U;

    for (uint8_t bus = 0U; bus < MOTOR_BUS_COUNT && slot < RCU_MOTOR_FB_SLOTS; ++bus) {
        for (uint8_t id = 1U; id <= MOTOR_MAX_PER_BUS && slot < RCU_MOTOR_FB_SLOTS; ++id) {
            const rs04_feedback_t *fb = motor_bus_get_feedback(bus, id);
            if (!fb || fb->motor_id == 0U) continue;

            rcu_motor_fb_slot_t *s = &out->slot[slot++];
            s->bus       = bus;
            s->motor_id  = fb->motor_id;
            s->pos_u16   = f_to_u16(fb->pos_rad,   -RS04_POS_MAX_RAD,  RS04_POS_MAX_RAD);
            s->vel_u16   = f_to_u16(fb->vel_rads,  -RS04_VEL_MAX_RADS, RS04_VEL_MAX_RADS);
            s->cur_u16    = f_to_u16(fb->torque_nm, -RS04_TRQ_MAX_NM, RS04_TRQ_MAX_NM);
            s->error_code = fb->fault_bits;
            s->mode_status = fb->mode_status;
        }
    }
    out->count = slot;
}

/* -----------------------------------------------------------------------
 * Motor command dispatch
 * ----------------------------------------------------------------------- */
void telem_pack_apply_motor_cmd(const rcu_motor_cmd_entry_t *entries, uint16_t count)
{
    for (uint16_t i = 0U; i < count; ++i) {
        const rcu_motor_cmd_entry_t *e = &entries[i];
        if (e->bus >= MOTOR_BUS_COUNT) continue;
        if (e->motor_id < 1U || e->motor_id > MOTOR_MAX_PER_BUS) continue;

        if (g_ctrl_mode == 1U) {
            /* CSP mode: send per-cycle position target via Type-18 param write */
            float pos_rad = u16_to_f(e->pos_u16, -RS04_POS_MAX_RAD, RS04_POS_MAX_RAD);
            motor_bus_send_param_write(e->bus, e->motor_id, RS04_PARAM_LOC_REF, pos_rad);
        } else {
            /* MIT mode (ctrl_mode=0): send Type-1 operation control frame */
            rs04_cmd_t cmd;
            cmd.pos_rad   = u16_to_f(e->pos_u16, -RS04_POS_MAX_RAD,  RS04_POS_MAX_RAD);
            cmd.vel_rads  = u16_to_f(e->vel_u16, -RS04_VEL_MAX_RADS, RS04_VEL_MAX_RADS);
            cmd.torque_nm = u16_to_f(e->trq_u16, -RS04_TRQ_MAX_NM,   RS04_TRQ_MAX_NM);
            cmd.kp        = ((float)e->kp_u8 / 255.0f) * RS04_KP_MAX;
            cmd.kd        = ((float)e->kd_u8 / 255.0f) * RS04_KD_MAX;
            motor_bus_send_cmd(e->bus, e->motor_id, &cmd);
        }
    }
}

/* -----------------------------------------------------------------------
 * Motor supervisory handler
 * ----------------------------------------------------------------------- */
extern void st_dbg_printf(const char *fmt, ...);

void telem_pack_apply_motor_supervisory(const rcu_motor_supervisory_t *sup)
{
    st_dbg_printf("[SUPV] enable_mask=0x%04X clr_fault=0x%04X ctrl=%u  (was 0x%04X)\r\n",
                  (unsigned)sup->enable_mask, (unsigned)sup->clear_fault_mask,
                  (unsigned)sup->ctrl_mode,   (unsigned)g_enabled_mask);
    g_ctrl_mode = sup->ctrl_mode;

    for (uint8_t n = 0U; n < 12U; ++n) {
        uint8_t  motor_id   = n + 1U;
        uint16_t bit        = (uint16_t)(1U << n);
        uint8_t  bus        = motor_id_to_bus(motor_id);
        bool     want_en    = (sup->enable_mask      & bit) != 0U;
        bool     was_en     = (g_enabled_mask         & bit) != 0U;
        bool     clr_fault  = (sup->clear_fault_mask  & bit) != 0U;

        if (want_en && !was_en) {
            /* --- Enable sequence (motor currently disabled) ---         */
            /* Step 0: optionally clear any latched fault first           */
            if (clr_fault) {
                motor_bus_send_enable(bus, motor_id, false, true);
            }
            if (sup->ctrl_mode == 0U) {
                /* MIT mode: write run_mode=0 BEFORE enable               */
                motor_bus_send_param_write(bus, motor_id,
                                           RS04_PARAM_RUN_MODE, 0.0f);
            } else if (sup->ctrl_mode == 1U) {
                /* CSP mode: write run_mode=5 BEFORE enable (s.4.3.4)    */
                motor_bus_send_param_write(bus, motor_id,
                                           RS04_PARAM_RUN_MODE, 5.0f);
            }
            /* Step 2: enable the motor */
            motor_bus_send_enable(bus, motor_id, true, false);
            if (sup->ctrl_mode == 1U) {
                /* Step 3: set speed limit after enable (CSP only)        */
                motor_bus_send_param_write(bus, motor_id,
                                           RS04_PARAM_LIMIT_SPD, 15.0f);
            }
        } else if (!want_en && was_en) {
            /* --- Disable (send Type-4 stop, no fault clear) ---         */
            motor_bus_send_enable(bus, motor_id, false, false);
        }
    }

    g_enabled_mask = sup->enable_mask;
}

/* -----------------------------------------------------------------------
 * Fast IMU packet — per-IMU mounting configuration
 *
 * For each body axis [X=fwd, Y=left, Z=up] specify which sensor axis feeds
 * it and what sign to apply.  src: 0=sensor_X, 1=sensor_Y, 2=sensor_Z.
 *
 * To change orientation: edit ONLY the two mount tables below.
 * ----------------------------------------------------------------------- */
typedef struct { uint8_t src; int8_t sign; } imu_axis_map_t;
typedef struct { imu_axis_map_t body[3]; } imu_mount_t;

/*
 * IMU0 — LSM6DSOX, chest
 *   sensor X → towards ground        (body -Z)
 *   sensor Y → out from chest fwd    (body +X)
 *   sensor Z → left→right shoulder   (body -Y)
 */
static const imu_mount_t IMU0_MOUNT = {{
    {1,  1},   /* body_x = +sensor_y */
    {2, -1},   /* body_y = -sensor_z */
    {0, -1},   /* body_z = -sensor_x */
}};

/*
 * IMU1 — LSM6DSOX, <location TBD>
 * Update src/sign entries to match physical mounting before use.
 *   sensor X → ?
 *   sensor Y → ?
 *   sensor Z → ?
 */
static const imu_mount_t IMU1_MOUNT = {{
    {0,  1},   /* body_x = +sensor_x  (PLACEHOLDER — update) */
    {1,  1},   /* body_y = +sensor_y  (PLACEHOLDER — update) */
    {2,  1},   /* body_z = +sensor_z  (PLACEHOLDER — update) */
}};

/* Apply a mount remap to one accel+gyro sample. */
static void apply_mount(const imu_mount_t *m,
                        const int16_t src_a[3], const int16_t src_g[3],
                        int16_t dst_a[3],       int16_t dst_g[3])
{
    for (int i = 0; i < 3; i++) {
        dst_a[i] = (int16_t)((int32_t)src_a[m->body[i].src] * m->body[i].sign);
        dst_g[i] = (int16_t)((int32_t)src_g[m->body[i].src] * m->body[i].sign);
    }
}

void telem_pack_imu_fast(rcu_imu_fast_t *out)
{
    memset(out, 0, sizeof(*out));
    out->tick_ms = HAL_GetTick();

    const imu_sample_t *s0 = imu_get_sample(0U);
    if (s0 && s0->valid) {
        apply_mount(&IMU0_MOUNT, s0->accel, s0->gyro, out->imu0_accel, out->imu0_gyro);
    }

    const imu_sample_t *s1 = imu_get_sample(1U);
    if (s1 && s1->valid) {
        apply_mount(&IMU1_MOUNT, s1->accel, s1->gyro, out->imu1_accel, out->imu1_gyro);
    }
}
