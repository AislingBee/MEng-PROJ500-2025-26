/*
 * eth_udp.c — Ethernet UDP transport, RCU runtime build
 *
 * lwIP raw-API usage:
 *   - g_tx_pcb: one connected UDP PCB for port 7700 and 7702 TX
 *   - g_rx_pcb: bound UDP PCB on port 7701, recv callback fills g_rx_buf
 *
 * TX path:  build header + payload into a pbuf, call udp_sendto()
 * RX path:  recv callback copies payload into a ring; eth_udp_tick()
 *           processes the ring and dispatches via telem_pack_apply_motor_cmd()
 *
 * Thread safety: all operations are in the superloop thread only.
 *
 * Note on MX_LWIP_Process():
 *   CubeMX generates this function in LWIP/App/lwip.c.  It drives
 *   ethernetif_input() and sys_check_timeouts().  It MUST be called
 *   frequently (every superloop pass) for lwIP to function.
 */
#include "eth_udp.h"
#include "telem_pack.h"
#include "mcan_pdu.h"
#include "imu.h"
#include "motor_bus.h"
#include "main.h"
#include "rcu_app.h"

/* lwIP headers */
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip.h"        /* MX_LWIP_Process() */

#include <stdbool.h>
#include <string.h>

extern void st_dbg_printf(const char *fmt, ...);
extern TIM_HandleTypeDef htim1;

/* -----------------------------------------------------------------------
 * Configuration
 * ----------------------------------------------------------------------- */
#define THOR_IP_A  192U
#define THOR_IP_B  168U
#define THOR_IP_C  100U
#define THOR_IP_D   20U   /* placeholder — update when confirmed */

#define PORT_TELEM_OUT    7700U
#define PORT_CMD_IN       7701U
#define PORT_SUPV_OUT     7702U

/* -----------------------------------------------------------------------
 * Module state
 * ----------------------------------------------------------------------- */
static struct udp_pcb *g_tx_pcb   = NULL;
static struct udp_pcb *g_rx_pcb   = NULL;

static ip_addr_t g_thor_addr;

/* Simple single-slot Rx staging buffer.
 * CRC/integrity is handled by the IP layer; we just store the last frame. */
#define RX_BUF_MAX  512U
static uint8_t  g_rx_buf[RX_BUF_MAX];
static uint16_t g_rx_len  = 0U;
static bool     g_rx_ready = false;

static uint8_t g_seq = 0U;

/* Saved at boot by eth_udp_init() for inclusion in debug replies */
static uint32_t g_boot_rsr = 0U;

/* Set by RCU_DBGCMD_FORCE_TELEM — checked in rcu_app.c superloop */
static volatile bool g_force_telem = false;

/* CAN loopback result: 0=untested, 1=right OK, 2=left OK, 3=both, 0xFF=fail */
static uint8_t g_can_loopback_result = 0U;
/* Forward declaration — defined below send_debug_reply */
static void handle_debug_cmd(const uint8_t *payload, uint16_t len);
/* -----------------------------------------------------------------------
 * lwIP receive callback
 * ----------------------------------------------------------------------- */
static void rx_callback(void *arg, struct udp_pcb *pcb,
                        struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)pcb; (void)addr; (void)port;

    if (p == NULL) return;

    uint16_t copy_len = (p->tot_len < RX_BUF_MAX) ? (uint16_t)p->tot_len : (uint16_t)RX_BUF_MAX;
    pbuf_copy_partial(p, g_rx_buf, copy_len, 0U);
    g_rx_len   = copy_len;
    g_rx_ready = true;

    pbuf_free(p);
}

/* -----------------------------------------------------------------------
 * Rx dispatch
 * ----------------------------------------------------------------------- */
static void process_rx(void)
{
    if (!g_rx_ready) return;
    g_rx_ready = false;

    if (g_rx_len < (uint16_t)RCU_PKT_HDR_SIZE) return;

    const rcu_pkt_hdr_t *hdr = (const rcu_pkt_hdr_t *)(void *)g_rx_buf;
    if (hdr->magic != RCU_PKT_MAGIC)             return;
    if ((uint16_t)(RCU_PKT_HDR_SIZE + hdr->len) > g_rx_len) return;

    const uint8_t *payload = g_rx_buf + RCU_PKT_HDR_SIZE;

    if (hdr->type == RCU_PKT_TYPE_MOTOR_CMD) {
        uint16_t n = hdr->len / (uint16_t)sizeof(rcu_motor_cmd_entry_t);
        telem_pack_apply_motor_cmd((const rcu_motor_cmd_entry_t *)(const void *)payload, n);
    } else if (hdr->type == RCU_PKT_TYPE_MOTOR_SUPV) {
        if (hdr->len >= (uint16_t)sizeof(rcu_motor_supervisory_t)) {
            telem_pack_apply_motor_supervisory(
                (const rcu_motor_supervisory_t *)(const void *)payload);
        }
    } else if (hdr->type == RCU_PKT_TYPE_DEBUG_CMD) {
        if (hdr->len >= 1U) {
            handle_debug_cmd(payload, hdr->len);
        }
    }
}

/* -----------------------------------------------------------------------
 * TX helper — build header + send pbuf
 * ----------------------------------------------------------------------- */
static void send_packet(uint16_t dst_port, uint8_t type,
                        const void *payload, uint16_t payload_len)
{
    if (!g_tx_pcb) return;

    uint16_t total = (uint16_t)RCU_PKT_HDR_SIZE + payload_len;
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, total, PBUF_RAM);
    if (!p) return;

    rcu_pkt_hdr_t hdr;
    hdr.magic = RCU_PKT_MAGIC;
    hdr.type  = type;
    hdr.seq   = g_seq++;
    hdr.len   = payload_len;

    uint16_t offset = 0U;
    pbuf_take_at(p, &hdr, (u16_t)RCU_PKT_HDR_SIZE, offset);
    offset += (uint16_t)RCU_PKT_HDR_SIZE;
    if (payload_len > 0U) {
        pbuf_take_at(p, payload, payload_len, offset);
    }

    udp_sendto(g_tx_pcb, p, &g_thor_addr, dst_port);
    pbuf_free(p);
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 * Debug reply builder — assembles rcu_debug_reply_t from live state
 * ----------------------------------------------------------------------- */
static void send_debug_reply(void)
{
    rcu_debug_reply_t rep;
    memset(&rep, 0, sizeof(rep));

    rep.uptime_ms   = HAL_GetTick();
    rep.boot_rsr    = g_boot_rsr;

    const imu_sample_t *s0 = imu_get_sample(0U);
    const imu_sample_t *s1 = imu_get_sample(1U);
    rep.imu0_valid = (s0 && s0->valid) ? 1U : 0U;
    rep.imu1_valid = (s1 && s1->valid) ? 1U : 0U;

    const pdu_telem_t *pdu = mcan_pdu_get_telem();
    rep.pdu_fpga_valid  = pdu->fpga_valid  ? 1U : 0U;
    rep.pdu_rails_valid = pdu->rails_valid ? 1U : 0U;
    rep.pdu_ssd_valid   = pdu->ssd_valid   ? 1U : 0U;
    rep.can_loopback    = g_can_loopback_result;

    uint32_t now = HAL_GetTick();
    rep.pdu_hb_age_ms = (pdu->hb_last_ms > 0U)
                        ? (now - pdu->hb_last_ms)
                        : 0xFFFFFFFFUL;

    send_packet(PORT_TELEM_OUT, RCU_PKT_TYPE_DEBUG_REPLY, &rep, (uint16_t)sizeof(rep));
}

/* -----------------------------------------------------------------------
 * Debug command handler
 * ----------------------------------------------------------------------- */
static void handle_debug_cmd(const uint8_t *payload, uint16_t len)
{
    switch (payload[0]) {

    case RCU_DBGCMD_PING:
        st_dbg_printf("[DBG] PING\r\n");
        send_debug_reply();
        break;


    case RCU_DBGCMD_BUZZ:
        st_dbg_printf("[DBG] BUZZ\r\n");
        __HAL_TIM_SET_AUTORELOAD(&htim1, 33332U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 16666U);
        __HAL_TIM_SET_COUNTER(&htim1, 0U);
        HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
        /* Forward to PDU simultaneously via AUX byte bit 3 */
        mcan_pdu_send_aux_cmd(0x08U);
        HAL_Delay(200U);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        send_debug_reply();
        break;

    case RCU_DBGCMD_LED_BLINK:
        st_dbg_printf("[DBG] LED_BLINK\r\n");
        /* Forward to PDU via AUX byte bit 4 */
        mcan_pdu_send_aux_cmd(0x10U);
        for (uint8_t i = 0U; i < 3U; i++) {
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
            HAL_Delay(150U);
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
            HAL_Delay(150U);
        }
        send_debug_reply();
        break;

    case RCU_DBGCMD_CAN_LOOPBACK:
        st_dbg_printf("[DBG] CAN_LOOPBACK\r\n");
        g_can_loopback_result = motor_bus_loopback_test();
        st_dbg_printf("[DBG] CAN loopback result=0x%02X\r\n", g_can_loopback_result);
        send_debug_reply();
        break;

    case RCU_DBGCMD_FORCE_TELEM:
        st_dbg_printf("[DBG] FORCE_TELEM\r\n");
        g_force_telem = true;
        /* reply will follow on next telem TX in rcu_app.c superloop */
        send_debug_reply();
        break;

    case RCU_DBGCMD_ASSERT_PDU_FAULT:
        st_dbg_printf("[DBG] ASSERT_PDU_FAULT\r\n");
        mcan_pdu_send_fault_req((len >= 2U) ? (payload[1] != 0U) : true);
        send_debug_reply();
        break;

    case RCU_DBGCMD_SOFT_RESET:
        st_dbg_printf("[DBG] SOFT_RESET\r\n");
        /* if (len >= 2U && payload[1]) { TODO: PDU reset via CAN } */
        send_debug_reply();
        HAL_Delay(10U);
        NVIC_SystemReset();
        break;

    case RCU_DBGCMD_SET_TELEM_RATE:
        st_dbg_printf("[DBG] SET_TELEM_RATE\r\n");
        if (len >= 2U && payload[1] > 0U) {
            rcu_app_set_telem_rate_ms(1000U / (uint32_t)payload[1]);
        }
        send_debug_reply();
        break;

    case RCU_DBGCMD_MOTOR_BUS_CTRL:
        st_dbg_printf("[DBG] MOTOR_BUS_CTRL\r\n");
        if (len >= 2U) {
            mcan_pdu_send_aux_cmd(payload[1] & 0x03U);
        }
        send_debug_reply();
        break;

    case RCU_DBGCMD_REQUEST_SUPV_DUMP:
        st_dbg_printf("[DBG] REQUEST_SUPV_DUMP\r\n");
        send_debug_reply();
        break;

    default:
        st_dbg_printf("[DBG] unknown subcmd 0x%02X\r\n", payload[0]);
        send_debug_reply();
        break;
    }
}

void eth_udp_init(void)
{
    g_boot_rsr = RCC->RSR;   /* snapshot before RMVF is cleared */
    IP4_ADDR(&g_thor_addr, THOR_IP_A, THOR_IP_B, THOR_IP_C, THOR_IP_D);

    /* TX PCB — unbound, address set per send */
    g_tx_pcb = udp_new();
    /* g_tx_pcb is used with udp_sendto(); no bind needed */

    /* RX PCB — bind to port 7701 */
    g_rx_pcb = udp_new();
    if (g_rx_pcb) {
        udp_bind(g_rx_pcb, IP_ADDR_ANY, PORT_CMD_IN);
        udp_recv(g_rx_pcb, rx_callback, NULL);
    }
}

void eth_udp_tick(uint32_t now_ms)
{
    (void)now_ms;
    MX_LWIP_Process();
    process_rx();
}

void eth_udp_send_telem(const rcu_telem_payload_t *payload)
{
    send_packet(PORT_TELEM_OUT, RCU_PKT_TYPE_SLOW_TELEM,
                payload, (uint16_t)sizeof(rcu_telem_payload_t));
}

void eth_udp_send_motor_fb(const rcu_motor_fb_payload_t *payload)
{
    send_packet(PORT_TELEM_OUT, RCU_PKT_TYPE_MOTOR_FB,
                payload, (uint16_t)sizeof(rcu_motor_fb_payload_t));
}

void eth_udp_send_imu_fast(const rcu_imu_fast_t *payload)
{
    send_packet(PORT_TELEM_OUT, RCU_PKT_TYPE_IMU_FAST,
                payload, (uint16_t)sizeof(rcu_imu_fast_t));
}

void eth_udp_send_supervision(const uint8_t *event_data, uint16_t len)
{
    send_packet(PORT_SUPV_OUT, RCU_PKT_TYPE_SUPERVISION, event_data, len);
}

bool eth_udp_consume_force_telem(void)
{
    if (!g_force_telem) return false;
    g_force_telem = false;
    return true;
}
