/*
 * eth_stack.c
 *
 * Minimal raw Ethernet stack for RCU board (STM32H723, LAN8742A PHY, RMII).
 * Static IP: 192.168.100.10 / 24
 * Handles: ARP request→reply, ICMP echo request→reply, UDP echo (port 7777).
 *
 * Buffer ownership:
 *   g_rx_data[][]  — DMA-accessible data buffers, D2 SRAM (.EthBuffSection)
 *   g_rx_desc[]    — ETH_BufferTypeDef chain nodes, CPU-only metadata (D1 SRAM)
 *   g_tx_buff[]    — TX staging buffer, D2 SRAM (.EthBuffSection)
 *
 * RX flow:
 *   HAL_ETH_RxAllocateCallback → assigns g_rx_data[N] to DMA descriptor
 *   HAL_ETH_RxLinkCallback     → links g_rx_desc[N] into ETH_BufferTypeDef chain
 *   HAL_ETH_ReadData           → returns chain via *pAppBuff; re-arms descriptor
 *   eth_stack_poll             → calls ReadData in loop; processes each frame
 */

#include "eth_stack.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ======================================================================
 *  External references
 * ====================================================================== */

extern ETH_HandleTypeDef heth;
extern void st_dbg_printf(const char *fmt, ...);

/* ======================================================================
 *  Config
 * ====================================================================== */

#ifndef ETH_MAX_PACKET_SIZE
#define ETH_MAX_PACKET_SIZE  1536U
#endif
#define UDP_ECHO_PORT        7777U

/* Locally administered unicast MAC — must match MACAddr[] in MX_ETH_Init */
static const uint8_t MY_MAC[6] = {0x02U, 0x00U, 0x52U, 0x43U, 0x55U, 0x01U};
static const uint8_t MY_IP[4]  = {192U, 168U, 100U, 10U};

/* LAN8742A Special Control/Status Register (reg 31) */
#define LAN8742A_PHYCSR             0x1FU
#define LAN8742A_PHYCSR_SPEED_100   (1U << 4)
#define LAN8742A_PHYCSR_FULL_DUPLEX (1U << 2)

/* ======================================================================
 *  DMA buffers — D2 SRAM, non-cacheable via MPU Region 1
 * ====================================================================== */

/* g_tx_buff used by both build modes (runtime eth_stack_init uses it too) */
static uint8_t g_tx_buff[ETH_MAX_PACKET_SIZE]
    __attribute__((section(".EthBuffSection"), aligned(32)));

/* g_rx_alloc_idx reset in eth_stack_init (both builds); callbacks that use it
 * are excluded from the runtime build via the guard below. */
static uint32_t g_rx_alloc_idx = 0U;

#if !defined(RCU_BUILD_MODE_RUNTIME)
/* g_rx_data / g_rx_desc are only accessed by the raw-stack RX callbacks,
 * which are also excluded from the runtime build. */
static uint8_t g_rx_data[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]
    __attribute__((section(".EthBuffSection"), aligned(32)));

/* ======================================================================
 *  CPU-side metadata — normal D1 SRAM, not DMA-accessed
 * ====================================================================== */

static ETH_BufferTypeDef g_rx_desc[ETH_RX_DESC_CNT];
#endif /* !RCU_BUILD_MODE_RUNTIME */

/* ======================================================================
 *  Runtime state
 * ====================================================================== */

static bool     g_eth_up      = false;
static bool     g_udp_echo_en = false;
static uint32_t g_phy_addr    = 0U;

eth_stats_t g_eth_stats = {0};

/* ======================================================================
 *  HAL RX callbacks — only used by the raw Ethernet selftest stack.
 *  In RUNTIME build ethernetif.c (lwIP) owns these callbacks.
 * ====================================================================== */
#if !defined(RCU_BUILD_MODE_RUNTIME)

void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
    *buff = g_rx_data[g_rx_alloc_idx];
    g_rx_alloc_idx = (g_rx_alloc_idx + 1U) % ETH_RX_DESC_CNT;
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd,
                             uint8_t *buff, uint16_t Length)
{
    /* Recover slot index from buffer address.  g_rx_data is a 2D array so
       each row is exactly ETH_MAX_PACKET_SIZE bytes apart. */
    uint32_t idx = (uint32_t)(buff - &g_rx_data[0][0]) / ETH_MAX_PACKET_SIZE;
    if (idx >= ETH_RX_DESC_CNT) return;  /* safety — should never happen */

    ETH_BufferTypeDef *node = &g_rx_desc[idx];
    node->buffer = buff;
    node->len    = Length;
    node->next   = NULL;

    if (*pEnd == NULL) {
        /* First (and for our MTU, only) segment */
        *pStart = node;
        *pEnd   = node;
    } else {
        /* Multi-segment: chain onto tail */
        ((ETH_BufferTypeDef *)*pEnd)->next = node;
        *pEnd = node;
    }
}

#endif /* !RCU_BUILD_MODE_RUNTIME */

/* ======================================================================
 *  Checksum helpers
 * ====================================================================== */

static uint16_t inet_checksum(const uint8_t *data, uint16_t len)
{
    uint32_t sum = 0U;
    while (len > 1U) {
        sum += ((uint32_t)data[0] << 8) | (uint32_t)data[1];
        data += 2; len -= 2U;
    }
    if (len != 0U) sum += (uint32_t)data[0] << 8;
    while (sum >> 16) sum = (sum & 0xFFFFU) + (sum >> 16);
    return (uint16_t)(~sum);
}

static uint16_t udp_checksum(const uint8_t *src_ip, const uint8_t *dst_ip,
                              const uint8_t *udp_hdr, uint16_t udp_len)
{
    /* Pseudo-header: src(4) + dst(4) + zero(1) + proto=17(1) + udp_len(2) */
    uint32_t sum = 0U;
    sum += ((uint32_t)src_ip[0] << 8) | (uint32_t)src_ip[1];
    sum += ((uint32_t)src_ip[2] << 8) | (uint32_t)src_ip[3];
    sum += ((uint32_t)dst_ip[0] << 8) | (uint32_t)dst_ip[1];
    sum += ((uint32_t)dst_ip[2] << 8) | (uint32_t)dst_ip[3];
    sum += 17U;
    sum += udp_len;
    const uint8_t *p = udp_hdr;
    uint16_t rem = udp_len;
    while (rem > 1U) {
        sum += ((uint32_t)p[0] << 8) | (uint32_t)p[1];
        p += 2; rem -= 2U;
    }
    if (rem != 0U) sum += (uint32_t)p[0] << 8;
    while (sum >> 16) sum = (sum & 0xFFFFU) + (sum >> 16);
    return (uint16_t)(~sum);
}

/* ======================================================================
 *  TX helper — uses g_tx_buff, no hardware checksum
 * ====================================================================== */

static bool eth_tx(uint16_t len)
{
    ETH_BufferTypeDef txbuf;
    txbuf.buffer = g_tx_buff;
    txbuf.len    = len;
    txbuf.next   = NULL;

    ETH_TxPacketConfigTypeDef cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.Attributes = ETH_TX_PACKETS_FEATURES_CRCPAD;
    cfg.CRCPadCtrl = ETH_CRC_PAD_INSERT;
    cfg.Length     = len;
    cfg.TxBuffer   = &txbuf;

    bool ok = (HAL_ETH_Transmit(&heth, &cfg, 100U) == HAL_OK);
    if (ok) g_eth_stats.tx_frames++;
    else    g_eth_stats.tx_fail++;
    return ok;
}

/* ======================================================================
 *  Frame layout constants
 * ====================================================================== */

/* Ethernet header */
#define ETH_OFF_DST   0U
#define ETH_OFF_SRC   6U
#define ETH_OFF_TYPE  12U
#define ETH_HDR_LEN   14U

/* ARP (after ETH header) */
#define ARP_OFF_OPCODE 6U
#define ARP_OFF_SHA    8U
#define ARP_OFF_SPA    14U
#define ARP_OFF_THA    18U
#define ARP_OFF_TPA    24U
#define ARP_FRAME_LEN  28U

/* IPv4 header (after ETH header, no options assumed) */
#define IP_OFF_VER_IHL   0U
#define IP_OFF_TOTAL_LEN 2U
#define IP_OFF_PROTO     9U
#define IP_OFF_CHECKSUM  10U
#define IP_OFF_SRC       12U
#define IP_OFF_DST       16U
#define IP_HDR_LEN       20U

/* ICMP (after IP header) */
#define ICMP_OFF_TYPE  0U
#define ICMP_OFF_CODE  1U
#define ICMP_OFF_CKSUM 2U
#define ICMP_HDR_LEN   4U

/* UDP (after IP header) */
#define UDP_OFF_SPORT  0U
#define UDP_OFF_DPORT  2U
#define UDP_OFF_LEN    4U
#define UDP_OFF_CKSUM  6U
#define UDP_HDR_LEN    8U

/* ======================================================================
 *  Protocol handlers
 * ====================================================================== */

static void handle_arp(const uint8_t *frame, uint16_t len)
{
    if (len < (uint16_t)(ETH_HDR_LEN + ARP_FRAME_LEN)) return;
    const uint8_t *arp = frame + ETH_HDR_LEN;

    /* Accept only ARP requests targeting our IP */
    uint16_t opcode = ((uint16_t)arp[ARP_OFF_OPCODE] << 8) | arp[ARP_OFF_OPCODE + 1U];
    if (opcode != 1U) return;
    if (memcmp(arp + ARP_OFF_TPA, MY_IP, 4) != 0) return;

    g_eth_stats.arp_rx++;

    uint8_t *tx   = g_tx_buff;
    uint8_t *rarp = tx + ETH_HDR_LEN;

    /* Ethernet header */
    memcpy(tx + ETH_OFF_DST,  arp + ARP_OFF_SHA, 6);
    memcpy(tx + ETH_OFF_SRC,  MY_MAC, 6);
    tx[ETH_OFF_TYPE]     = 0x08U;
    tx[ETH_OFF_TYPE + 1] = 0x06U;

    /* ARP reply payload */
    rarp[0] = 0x00U; rarp[1] = 0x01U;   /* HW type: Ethernet */
    rarp[2] = 0x08U; rarp[3] = 0x00U;   /* Protocol: IPv4 */
    rarp[4] = 6U;    rarp[5] = 4U;      /* HW addr len, proto addr len */
    rarp[ARP_OFF_OPCODE]     = 0x00U;
    rarp[ARP_OFF_OPCODE + 1] = 0x02U;   /* opcode: reply */
    memcpy(rarp + ARP_OFF_SHA, MY_MAC, 6);
    memcpy(rarp + ARP_OFF_SPA, MY_IP,  4);
    memcpy(rarp + ARP_OFF_THA, arp + ARP_OFF_SHA, 6);
    memcpy(rarp + ARP_OFF_TPA, arp + ARP_OFF_SPA, 4);

    if (eth_tx(ETH_HDR_LEN + ARP_FRAME_LEN)) g_eth_stats.arp_tx++;
}

static void handle_icmp(const uint8_t *frame, uint16_t len)
{
    if (len < (uint16_t)(ETH_HDR_LEN + IP_HDR_LEN + ICMP_HDR_LEN)) return;
    const uint8_t *ip   = frame + ETH_HDR_LEN;
    if (ip[IP_OFF_VER_IHL] != 0x45U) return;   /* IPv4, no IP options */

    uint16_t ip_total = ((uint16_t)ip[IP_OFF_TOTAL_LEN] << 8) | ip[IP_OFF_TOTAL_LEN + 1U];
    if (ip_total < (uint16_t)(IP_HDR_LEN + ICMP_HDR_LEN)) return;

    const uint8_t *icmp = ip + IP_HDR_LEN;
    if (icmp[ICMP_OFF_TYPE] != 8U) return;   /* echo request only */
    if (icmp[ICMP_OFF_CODE] != 0U) return;

    g_eth_stats.icmp_rx++;

    uint16_t frame_total = (uint16_t)(ETH_HDR_LEN + ip_total);
    if (frame_total > ETH_MAX_PACKET_SIZE) return;

    memcpy(g_tx_buff, frame, frame_total);
    uint8_t *tip   = g_tx_buff + ETH_HDR_LEN;
    uint8_t *ticmp = tip + IP_HDR_LEN;

    /* Swap Ethernet src/dst */
    memcpy(g_tx_buff + ETH_OFF_DST, frame + ETH_OFF_SRC, 6);
    memcpy(g_tx_buff + ETH_OFF_SRC, MY_MAC, 6);

    /* Swap IP src/dst */
    uint8_t tmp[4];
    memcpy(tmp,              tip + IP_OFF_SRC, 4);
    memcpy(tip + IP_OFF_SRC, tip + IP_OFF_DST, 4);
    memcpy(tip + IP_OFF_DST, tmp,              4);

    /* Recompute IP header checksum */
    tip[IP_OFF_CHECKSUM]     = 0U;
    tip[IP_OFF_CHECKSUM + 1] = 0U;
    uint16_t iphck = inet_checksum(tip, IP_HDR_LEN);
    tip[IP_OFF_CHECKSUM]     = (uint8_t)(iphck >> 8);
    tip[IP_OFF_CHECKSUM + 1] = (uint8_t)(iphck);

    /* Change ICMP type to echo reply, recompute ICMP checksum */
    ticmp[ICMP_OFF_TYPE]      = 0U;
    ticmp[ICMP_OFF_CKSUM]     = 0U;
    ticmp[ICMP_OFF_CKSUM + 1] = 0U;
    uint16_t icmphck = inet_checksum(ticmp, (uint16_t)(ip_total - IP_HDR_LEN));
    ticmp[ICMP_OFF_CKSUM]     = (uint8_t)(icmphck >> 8);
    ticmp[ICMP_OFF_CKSUM + 1] = (uint8_t)(icmphck);

    if (eth_tx(frame_total)) g_eth_stats.icmp_tx++;
}

static void handle_udp(const uint8_t *frame, uint16_t len)
{
    if (!g_udp_echo_en) return;
    if (len < (uint16_t)(ETH_HDR_LEN + IP_HDR_LEN + UDP_HDR_LEN)) return;

    const uint8_t *ip = frame + ETH_HDR_LEN;
    if (ip[IP_OFF_VER_IHL] != 0x45U) return;
    if (ip[IP_OFF_PROTO]   != 17U)   return;

    uint16_t ip_total = ((uint16_t)ip[IP_OFF_TOTAL_LEN] << 8) | ip[IP_OFF_TOTAL_LEN + 1U];
    if (ip_total < (uint16_t)(IP_HDR_LEN + UDP_HDR_LEN)) return;

    const uint8_t *udp = ip + IP_HDR_LEN;
    uint16_t dport = ((uint16_t)udp[UDP_OFF_DPORT] << 8) | udp[UDP_OFF_DPORT + 1U];
    if (dport != UDP_ECHO_PORT) return;

    g_eth_stats.udp_rx++;

    uint16_t frame_total = (uint16_t)(ETH_HDR_LEN + ip_total);
    if (frame_total > ETH_MAX_PACKET_SIZE) return;

    memcpy(g_tx_buff, frame, frame_total);
    uint8_t *tip  = g_tx_buff + ETH_HDR_LEN;
    uint8_t *tudp = tip + IP_HDR_LEN;

    /* Swap Ethernet src/dst */
    memcpy(g_tx_buff + ETH_OFF_DST, frame + ETH_OFF_SRC, 6);
    memcpy(g_tx_buff + ETH_OFF_SRC, MY_MAC, 6);

    /* Swap IP src/dst */
    uint8_t tmp_ip[4];
    memcpy(tmp_ip,           tip + IP_OFF_SRC, 4);
    memcpy(tip + IP_OFF_SRC, tip + IP_OFF_DST, 4);
    memcpy(tip + IP_OFF_DST, tmp_ip,           4);

    /* Swap UDP src/dst ports */
    uint8_t tmp_port[2];
    memcpy(tmp_port,                tudp + UDP_OFF_SPORT, 2);
    memcpy(tudp + UDP_OFF_SPORT,    tudp + UDP_OFF_DPORT, 2);
    memcpy(tudp + UDP_OFF_DPORT,    tmp_port,             2);

    /* Recompute IP header checksum */
    tip[IP_OFF_CHECKSUM]     = 0U;
    tip[IP_OFF_CHECKSUM + 1] = 0U;
    uint16_t iphck = inet_checksum(tip, IP_HDR_LEN);
    tip[IP_OFF_CHECKSUM]     = (uint8_t)(iphck >> 8);
    tip[IP_OFF_CHECKSUM + 1] = (uint8_t)(iphck);

    /* Recompute UDP checksum */
    uint16_t udp_len = ((uint16_t)tudp[UDP_OFF_LEN] << 8) | tudp[UDP_OFF_LEN + 1U];
    tudp[UDP_OFF_CKSUM]     = 0U;
    tudp[UDP_OFF_CKSUM + 1] = 0U;
    uint16_t udphck = udp_checksum(tip + IP_OFF_SRC, tip + IP_OFF_DST, tudp, udp_len);
    tudp[UDP_OFF_CKSUM]     = (uint8_t)(udphck >> 8);
    tudp[UDP_OFF_CKSUM + 1] = (uint8_t)(udphck);

    if (eth_tx(frame_total)) g_eth_stats.udp_tx++;
}

static void eth_process_frame(const uint8_t *buf, uint16_t len)
{
    if (len < ETH_HDR_LEN) { g_eth_stats.rx_discard++; return; }

    /* Accept only unicast-to-us or broadcast */
    static const uint8_t bcast[6] = {0xFFU,0xFFU,0xFFU,0xFFU,0xFFU,0xFFU};
    if (memcmp(buf + ETH_OFF_DST, MY_MAC, 6) != 0 &&
        memcmp(buf + ETH_OFF_DST, bcast,  6) != 0) {
        g_eth_stats.rx_discard++;
        return;
    }

    g_eth_stats.rx_frames++;
    uint16_t etype = ((uint16_t)buf[ETH_OFF_TYPE] << 8) | buf[ETH_OFF_TYPE + 1U];

    if (etype == 0x0806U) {
        handle_arp(buf, len);
    } else if (etype == 0x0800U) {
        if (len < (uint16_t)(ETH_HDR_LEN + IP_HDR_LEN)) {
            g_eth_stats.rx_discard++; return;
        }
        const uint8_t *ip = buf + ETH_HDR_LEN;
        if (memcmp(ip + IP_OFF_DST, MY_IP, 4) != 0) {
            g_eth_stats.rx_discard++; return;
        }
        uint8_t proto = ip[IP_OFF_PROTO];
        if (proto == 1U)       handle_icmp(buf, len);
        else if (proto == 17U) handle_udp(buf, len);
        else                   g_eth_stats.rx_discard++;
    } else {
        g_eth_stats.rx_discard++;
    }
}

/* ======================================================================
 *  PHY helpers (MDIO via HAL, direct — st_eth_phy_* are static elsewhere)
 * ====================================================================== */

static bool phy_read(uint32_t addr, uint32_t reg, uint32_t *val)
{
    return HAL_ETH_ReadPHYRegister(&heth, addr, reg, val) == HAL_OK;
}

static bool phy_detect(uint32_t *out_addr)
{
    uint32_t v1, v2;
    for (uint32_t a = 0U; a < 32U; ++a) {
        v1 = 0U; v2 = 0U;
        if (!phy_read(a, 0x02U, &v1)) continue;
        if (!phy_read(a, 0x03U, &v2)) continue;
        if (v1 == 0U || v1 == 0xFFFFU) continue;
        if (v2 == 0U || v2 == 0xFFFFU) continue;
        *out_addr = a;
        return true;
    }
    return false;
}

/* ======================================================================
 *  Public API
 * ====================================================================== */

void eth_stack_init(void)
{
    if (g_eth_up) return;

    /* 1. PHY hardware reset */
    HAL_GPIO_WritePin(ETH_NRST_GPIO_Port, ETH_NRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20U);
    HAL_GPIO_WritePin(ETH_NRST_GPIO_Port, ETH_NRST_Pin, GPIO_PIN_SET);
    HAL_Delay(100U);

    /* 2. Detect PHY on MDIO bus */
    uint32_t phy = 0U;
    uint32_t deadline = HAL_GetTick() + 500U;
    while (!phy_detect(&phy)) {
        if ((int32_t)(HAL_GetTick() - deadline) >= 0) {
            st_dbg_printf("[ETH] no PHY detected\r\n");
            return;
        }
        HAL_Delay(10U);
    }
    g_phy_addr = phy;

    /* 3. Wait for autoneg complete + link up.
     *    BMSR bit 2 (Link Status) is latch-low per IEEE 802.3: cleared on
     *    transient link drop, stays 0 until read.  Double-read to flush any
     *    stale latch from the PHY reset period. */
    deadline = HAL_GetTick() + 3000U;
    bool link_up = false;
    while ((int32_t)(HAL_GetTick() - deadline) < 0) {
        uint32_t bmsr1 = 0U, bmsr2 = 0U;
        (void)phy_read(phy, 0x01U, &bmsr1);   /* first read: clears latch */
        (void)phy_read(phy, 0x01U, &bmsr2);   /* second read: actual state */
        /* bit 5 = Autoneg Complete, bit 2 = Link Status */
        if ((bmsr2 & 0x0020U) && (bmsr2 & 0x0004U)) {
            link_up = true;
            break;
        }
        HAL_Delay(100U);
    }
    if (!link_up) {
        st_dbg_printf("[ETH] autoneg timeout\r\n");
        return;
    }

    /* 4. Read LAN8742A PHYCSR (reg 31) for negotiated speed/duplex.
     *    Fail explicitly — no fallback to a guessed 100/Full. */
    uint32_t phycsr = 0U;
    if (!phy_read(phy, LAN8742A_PHYCSR, &phycsr)) {
        st_dbg_printf("[ETH] PHY speed/duplex read failed\r\n");
        return;
    }
    uint32_t spd = (phycsr & LAN8742A_PHYCSR_SPEED_100)
                   ? ETH_SPEED_100M : ETH_SPEED_10M;
    uint32_t dup = (phycsr & LAN8742A_PHYCSR_FULL_DUPLEX)
                   ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;

    /* 5. Apply speed/duplex to MAC */
    ETH_MACConfigTypeDef macconf;
    if (HAL_ETH_GetMACConfig(&heth, &macconf) != HAL_OK) {
        st_dbg_printf("[ETH] GetMACConfig failed\r\n");
        return;
    }
    macconf.Speed      = spd;
    macconf.DuplexMode = dup;
    if (HAL_ETH_SetMACConfig(&heth, &macconf) != HAL_OK) {
        st_dbg_printf("[ETH] SetMACConfig failed\r\n");
        return;
    }

    /* 6. Reset alloc index so slots 0..N-1 map to descriptors 0..N-1 */
    g_rx_alloc_idx = 0U;

    /* 7. Start ETH — triggers RxAllocateCallback for each descriptor */
    if (HAL_ETH_Start(&heth) != HAL_OK) {
        st_dbg_printf("[ETH] HAL_ETH_Start failed\r\n");
        return;
    }

    g_eth_up = true;
    st_dbg_printf("[ETH] up  IP 192.168.100.10  MAC 02:00:52:43:55:01  %s/%s\r\n",
                  (spd == ETH_SPEED_100M)     ? "100M" : "10M",
                  (dup == ETH_FULLDUPLEX_MODE) ? "Full" : "Half");
}

void eth_stack_poll(void)
{
    if (!g_eth_up) return;
    void *appbuf = NULL;
    /* Drain all pending received frames */
    while (HAL_ETH_ReadData(&heth, &appbuf) == HAL_OK) {
        ETH_BufferTypeDef *p = (ETH_BufferTypeDef *)appbuf;
        if (p != NULL) {
            eth_process_frame(p->buffer, (uint16_t)p->len);
        }
        appbuf = NULL;
    }
}

void eth_stack_stop(void)
{
    if (!g_eth_up) return;
    (void)HAL_ETH_Stop(&heth);
    HAL_GPIO_WritePin(ETH_NRST_GPIO_Port, ETH_NRST_Pin, GPIO_PIN_RESET);
    g_eth_up = false;
    st_dbg_printf("[ETH] down\r\n");
}

bool eth_stack_is_up(void) { return g_eth_up; }

void eth_stack_udp_echo_set(bool enable)
{
    g_udp_echo_en = enable;
    st_dbg_printf("[ETH] UDP echo %s (port %u)\r\n",
                  enable ? "on" : "off", (unsigned)UDP_ECHO_PORT);
}

void eth_stack_print_ip(void)
{
    st_dbg_printf("[ETH] IP=192.168.100.10/24  MAC=02:00:52:43:55:01  PHY@%lu  up=%u\r\n",
                  (unsigned long)g_phy_addr, g_eth_up ? 1U : 0U);
}

void eth_stack_print_stats(void)
{
    st_dbg_printf("[ETH] rx=%lu tx=%lu"
                  "  arp_rx=%lu arp_tx=%lu"
                  "  icmp_rx=%lu icmp_tx=%lu"
                  "  udp_rx=%lu udp_tx=%lu"
                  "  discard=%lu txfail=%lu\r\n",
                  (unsigned long)g_eth_stats.rx_frames,
                  (unsigned long)g_eth_stats.tx_frames,
                  (unsigned long)g_eth_stats.arp_rx,
                  (unsigned long)g_eth_stats.arp_tx,
                  (unsigned long)g_eth_stats.icmp_rx,
                  (unsigned long)g_eth_stats.icmp_tx,
                  (unsigned long)g_eth_stats.udp_rx,
                  (unsigned long)g_eth_stats.udp_tx,
                  (unsigned long)g_eth_stats.rx_discard,
                  (unsigned long)g_eth_stats.tx_fail);
}
