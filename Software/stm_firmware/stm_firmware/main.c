/*
 * main.c
 * STM32 Ethernet-to-CAN bridge for RobStride RS04 motor control.
 *
 * Network protocol (UDP, port 7777, static IP 192.168.1.100):
 *   CMD 0xNN <q> <kp> <kd> <tau>\n
 *   CMD <q> <kp> <kd> <tau>\n
 *   ID?\n
 *   ZERO\n
 *   STOP\n
 *
 * Responses (UDP back to sender):
 *   FBK 0xNN <q> <q_dot>\n
 *   ID 0xNN\n
 *   ERR ...\n
 */

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan1;
ETH_HandleTypeDef heth;

/* Network configuration */
#define ETH_UDP_PORT        7777U
#define ETH_PHY_ADDR        0U

/* DMA descriptor counts and buffer sizes */
#define ETH_RXBUFNB         4U
#define ETH_TXBUFNB         4U
#define ETH_RX_BUF_SIZE     1524U
#define ETH_TX_BUF_SIZE     1524U

/* Ethernet frame offsets */
#define ETH_HEADER_SIZE       14U
#define ETHERTYPE_ARP         0x0806U
#define ETHERTYPE_IP          0x0800U
#define ARP_PACKET_SIZE       28U
#define ARP_OPER_REQUEST      0x0001U
#define IP_PROTO_UDP          17U
#define IP_HEADER_MIN_SIZE    20U

#define LINE_BUF_SIZE 128U
#define DEFAULT_MOTOR_CAN_ID 0x7FU

#define RS04_COMM_GET_ID            0x00U
#define RS04_COMM_MOTION_CONTROL    0x01U
#define RS04_COMM_MOTOR_FEEDBACK    0x02U
#define RS04_COMM_MOTOR_ENABLE      0x03U
#define RS04_COMM_MOTOR_STOP        0x04U
#define RS04_COMM_SET_POS_ZERO      0x06U
#define RS04_COMM_SET_SINGLE_PARAM  0x12U
#define RS04_RUN_MODE_INDEX         0x7005U
#define RS04_MOVE_CONTROL_MODE      0x00U
#define RS04_MASTER_CAN_ID          0x00U

#define P_MIN  (-12.5f)
#define P_MAX  (12.5f)
#define V_MIN  (-44.0f)
#define V_MAX  (44.0f)
#define KP_MIN (0.0f)
#define KP_MAX (500.0f)
#define KD_MIN (0.0f)
#define KD_MAX (5.0f)
#define T_MIN  (-17.0f)
#define T_MAX  (17.0f)

/* DMA descriptors and buffers (aligned for DMA access) */
static ETH_DMADescTypeDef DMARxDscrTab[ETH_RXBUFNB] __attribute__((aligned(4)));
static ETH_DMADescTypeDef DMATxDscrTab[ETH_TXBUFNB] __attribute__((aligned(4)));
static uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __attribute__((aligned(4)));

static const uint8_t my_mac[6] = {0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x01U};
static const uint8_t my_ip[4]  = {192U, 168U, 1U, 100U};

/* Last host that sent a UDP packet — responses go here */
static uint8_t host_mac[6]  = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
static uint8_t host_ip[4]   = {192U, 168U, 1U, 1U};
static uint16_t host_port   = 0U;
static bool host_known       = false;

static bool motor_mode_enabled = false;
static uint32_t motor_can_id   = DEFAULT_MOTOR_CAN_ID;

static bool can_loopback_ok = false;

static char line_buf[LINE_BUF_SIZE];

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_CAN1_Init(void);
static void service_eth_rx(void);
static void service_can_rx(void);
static void process_line(const char *line);
static void eth_send_udp(const char *data, uint16_t len);
static void send_text(const char *str);
static void send_feedback(uint32_t can_id, float q, float q_dot);
static void handle_arp(const uint8_t *frame, uint16_t frame_len);
static void handle_ip(const uint8_t *frame, uint16_t frame_len);
static void handle_udp(const uint8_t *udp, uint16_t udp_len,
                       const uint8_t *src_mac, const uint8_t *src_ip);
static void send_motor_command(uint32_t can_id, float q, float qd, float kp, float kd, float tau);
static bool can_send_frame(uint32_t ext_id, const uint8_t *data, uint8_t dlc);

/* Run CAN loopback self-test. Returns 1 on pass, 0 on fail.
 * Call immediately after MX_CAN1_Init(); restores normal mode before returning. */
static int loopback_test(void)
{
    CAN_FilterTypeDef can_filter = {0};

    HAL_CAN_Stop(&hcan1);
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) { goto restore; }

    can_filter.FilterBank          = 0;
    can_filter.FilterMode          = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale         = CAN_FILTERSCALE_32BIT;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation    = ENABLE;
    can_filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter);

    if (HAL_CAN_Start(&hcan1) != HAL_OK) { goto restore; }

    {
        CAN_TxHeaderTypeDef txH = {0};
        uint32_t mb = 0U;
        uint8_t td[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};
        txH.ExtId = 0x12345U; txH.IDE = CAN_ID_EXT;
        txH.RTR   = CAN_RTR_DATA; txH.DLC = 8U;
        txH.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(&hcan1, &txH, td, &mb) != HAL_OK) { goto restore; }

        uint32_t t0 = HAL_GetTick();
        while (HAL_CAN_IsTxMessagePending(&hcan1, mb)) {
            if (HAL_GetTick() - t0 > 100U) { goto restore; }
        }

        t0 = HAL_GetTick();
        while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U) {
            if (HAL_GetTick() - t0 > 100U) { goto restore; }
        }

        CAN_RxHeaderTypeDef rxH;
        uint8_t rd[8];
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxH, rd);

        if (rxH.ExtId == 0x12345U && rd[0] == 0xDEU && rd[3] == 0xEFU) {
            /* Restore normal mode */
            HAL_CAN_Stop(&hcan1);
            hcan1.Init.Mode = CAN_MODE_NORMAL;
            HAL_CAN_Init(&hcan1);
            HAL_CAN_ConfigFilter(&hcan1, &can_filter);
            HAL_CAN_Start(&hcan1);
            return 1;
        }
    }

restore:
    HAL_CAN_Stop(&hcan1);
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    HAL_CAN_Init(&hcan1);
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = 0; f.FilterMode = CAN_FILTERMODE_IDMASK;
        f.FilterScale = CAN_FILTERSCALE_32BIT;
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        f.FilterActivation = ENABLE; f.SlaveStartFilterBank = 14;
        HAL_CAN_ConfigFilter(&hcan1, &f);
    }
    HAL_CAN_Start(&hcan1);
    return 0;
}

static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    const float span = x_max - x_min;
    const float offset = x_min;
    const uint32_t max_int = (1UL << bits) - 1UL;
    return (uint32_t)(((x - offset) * (float)max_int) / span);
}

static float uint_to_float(uint32_t x_int, float x_min, float x_max, uint8_t bits)
{
    const float span = x_max - x_min;
    const float offset = x_min;
    const uint32_t max_int = (1UL << bits) - 1UL;
    return (((float)x_int) * span / (float)max_int) + offset;
}

/* One's-complement IP checksum */
static uint16_t ip_checksum(const uint8_t *buf, uint16_t len)
{
    uint32_t sum = 0U;
    while (len > 1U) {
        sum += ((uint32_t)buf[0] << 8) | buf[1];
        buf += 2;
        len -= 2U;
    }
    if (len == 1U) {
        sum += (uint32_t)buf[0] << 8;
    }
    while ((sum >> 16) != 0U) {
        sum = (sum & 0xFFFFUL) + (sum >> 16);
    }
    return (uint16_t)(~sum);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();

    /* CAN loopback self-test — validates the CAN peripheral before going live.
     * 3 fast LED blinks = PASS; 1 long blink = FAIL. */
    can_loopback_ok = (loopback_test() == 1);
    if (can_loopback_ok) {
        for (int i = 0; i < 6; i++) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
            HAL_Delay(80U);
        }
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_Delay(600U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    }

    MX_ETH_Init();

    /* Report boot status on first UDP packet from host */
    bool boot_reported = false;

    while (1) {
        service_eth_rx();
        service_can_rx();

        if (!boot_reported && host_known) {
            boot_reported = true;
            if (can_loopback_ok) {
                send_text("BOOT CAN loopback PASS\r\n");
            } else {
                send_text("BOOT CAN loopback FAIL — check transceiver and bus termination\r\n");
            }
        }
    }
}

/* Poll ETH DMA for a received frame, dispatch to handlers, release descriptors */
static void service_eth_rx(void)
{
    if (HAL_ETH_GetReceivedFrame(&heth) != HAL_OK) {
        return;
    }

    const uint8_t *frame = (const uint8_t *)heth.RxFrameInfos.buffer;
    const uint16_t flen  = (uint16_t)heth.RxFrameInfos.length;

    if (flen >= ETH_HEADER_SIZE) {
        const uint16_t etype = (uint16_t)(((uint16_t)frame[12] << 8) | frame[13]);
        if (etype == ETHERTYPE_ARP && flen >= (ETH_HEADER_SIZE + ARP_PACKET_SIZE)) {
            handle_arp(frame, flen);
        } else if (etype == ETHERTYPE_IP && flen >= (ETH_HEADER_SIZE + IP_HEADER_MIN_SIZE)) {
            handle_ip(frame, flen);
        }
    }

    /* Release descriptors back to DMA */
    ETH_DMADescTypeDef *desc = heth.RxFrameInfos.FSRxDesc;
    for (uint32_t i = 0U; i < heth.RxFrameInfos.SegCount; ++i) {
        desc->Status |= ETH_DMARXDESC_OWN;
        desc = (ETH_DMADescTypeDef *)(desc->Buffer2NextDescAddr);
    }
    heth.RxFrameInfos.SegCount = 0U;

    /* Resume DMA reception if suspended */
    if ((heth.Instance->DMASR & ETH_DMASR_RBUS) != 0U) {
        heth.Instance->DMASR = ETH_DMASR_RBUS;
        heth.Instance->DMARPDR = 0U;
    }
}

/* Respond to ARP requests for our IP */
static void handle_arp(const uint8_t *frame, uint16_t frame_len)
{
    (void)frame_len;
    const uint8_t *arp = frame + ETH_HEADER_SIZE;
    /* ARP layout (after Ethernet header):
       0-1: HTYPE  2-3: PTYPE  4: HLEN  5: PLEN  6-7: OPER
       8-13: SHA  14-17: SPA  18-23: THA  24-27: TPA */
    const uint16_t oper = (uint16_t)(((uint16_t)arp[6] << 8) | arp[7]);
    if (oper != ARP_OPER_REQUEST) {
        return;
    }
    if (memcmp(arp + 24, my_ip, 4) != 0) {
        return;
    }

    uint8_t *tx = (uint8_t *)heth.TxDesc->Buffer1Addr;

    /* Ethernet header */
    memcpy(tx,      arp + 8, 6);   /* dst = sender MAC */
    memcpy(tx + 6,  my_mac,  6);
    tx[12] = 0x08U; tx[13] = 0x06U;

    /* ARP reply */
    uint8_t *rep = tx + ETH_HEADER_SIZE;
    rep[0] = 0x00U; rep[1] = 0x01U; /* HTYPE Ethernet */
    rep[2] = 0x08U; rep[3] = 0x00U; /* PTYPE IP */
    rep[4] = 6U;    rep[5] = 4U;    /* HLEN, PLEN */
    rep[6] = 0x00U; rep[7] = 0x02U; /* OPER reply */
    memcpy(rep + 8,  my_mac,   6);  /* SHA = our MAC */
    memcpy(rep + 14, my_ip,    4);  /* SPA = our IP */
    memcpy(rep + 18, arp + 8,  6);  /* THA = requester MAC */
    memcpy(rep + 22, arp + 14, 4);  /* TPA = requester IP */

    HAL_ETH_TransmitFrame(&heth, ETH_HEADER_SIZE + ARP_PACKET_SIZE);
}

/* Dispatch IPv4 frames (UDP only) */
static void handle_ip(const uint8_t *frame, uint16_t frame_len)
{
    const uint8_t *ip       = frame + ETH_HEADER_SIZE;
    const uint16_t ip_total = (uint16_t)(((uint16_t)ip[2] << 8) | ip[3]);
    const uint8_t  ihl      = (uint8_t)((ip[0] & 0x0FU) * 4U);

    if (ihl < IP_HEADER_MIN_SIZE) {
        return;
    }
    if ((uint32_t)ETH_HEADER_SIZE + ip_total > frame_len) {
        return;
    }
    if (memcmp(ip + 16, my_ip, 4) != 0) {
        return;
    }
    if (ip[9] == IP_PROTO_UDP) {
        handle_udp(ip + ihl, (uint16_t)(ip_total - ihl),
                   frame + 6, ip + 12);
    }
}

/* Parse UDP datagram, remember host, process command lines */
static void handle_udp(const uint8_t *udp, uint16_t udp_len,
                       const uint8_t *src_mac, const uint8_t *src_ip)
{
    if (udp_len < 8U) {
        return;
    }
    const uint16_t dst_port    = (uint16_t)(((uint16_t)udp[2] << 8) | udp[3]);
    const uint16_t payload_len = (uint16_t)((((uint16_t)udp[4] << 8) | udp[5]) - 8U);

    if (dst_port != ETH_UDP_PORT) {
        return;
    }
    if (payload_len == 0U || payload_len >= LINE_BUF_SIZE) {
        return;
    }

    memcpy(host_mac, src_mac, 6);
    memcpy(host_ip,  src_ip,  4);
    host_port  = (uint16_t)(((uint16_t)udp[0] << 8) | udp[1]);
    host_known = true;

    const uint8_t *payload = udp + 8U;
    uint16_t i = 0U;
    uint16_t line_len = 0U;
    while (i < payload_len) {
        const char c = (char)payload[i++];
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            if (line_len > 0U) {
                line_buf[line_len] = '\0';
                process_line(line_buf);
                line_len = 0U;
            }
            continue;
        }
        if (line_len < (LINE_BUF_SIZE - 1U)) {
            line_buf[line_len++] = c;
        }
    }
    if (line_len > 0U) {
        line_buf[line_len] = '\0';
        process_line(line_buf);
    }
}

/* Send a UDP datagram to the last known host */
static void eth_send_udp(const char *data, uint16_t payload_len)
{
    if (!host_known) {
        return;
    }

    uint32_t timeout = 100000U;
    while ((heth.TxDesc->Status & ETH_DMATXDESC_OWN) != 0U) {
        if (--timeout == 0U) {
            return;
        }
    }

    const uint16_t udp_len   = (uint16_t)(8U + payload_len);
    const uint16_t ip_len    = (uint16_t)(IP_HEADER_MIN_SIZE + udp_len);
    const uint16_t frame_len = (uint16_t)(ETH_HEADER_SIZE + ip_len);

    uint8_t *tx = (uint8_t *)heth.TxDesc->Buffer1Addr;

    /* Ethernet header */
    memcpy(tx,     host_mac, 6);
    memcpy(tx + 6, my_mac,   6);
    tx[12] = 0x08U; tx[13] = 0x00U;

    /* IP header */
    uint8_t *ip = tx + ETH_HEADER_SIZE;
    ip[0]  = 0x45U;
    ip[1]  = 0x00U;
    ip[2]  = (uint8_t)(ip_len >> 8);
    ip[3]  = (uint8_t)(ip_len & 0xFFU);
    ip[4]  = 0x00U; ip[5]  = 0x00U;     /* ID */
    ip[6]  = 0x00U; ip[7]  = 0x00U;     /* flags/fragment */
    ip[8]  = 64U;                         /* TTL */
    ip[9]  = IP_PROTO_UDP;
    ip[10] = 0x00U; ip[11] = 0x00U;     /* checksum filled below */
    memcpy(ip + 12, my_ip,   4);
    memcpy(ip + 16, host_ip, 4);
    const uint16_t ip_csum = ip_checksum(ip, IP_HEADER_MIN_SIZE);
    ip[10] = (uint8_t)(ip_csum >> 8);
    ip[11] = (uint8_t)(ip_csum & 0xFFU);

    /* UDP header */
    uint8_t *udph = ip + IP_HEADER_MIN_SIZE;
    udph[0] = (uint8_t)(ETH_UDP_PORT >> 8);
    udph[1] = (uint8_t)(ETH_UDP_PORT & 0xFFU);
    udph[2] = (uint8_t)(host_port >> 8);
    udph[3] = (uint8_t)(host_port & 0xFFU);
    udph[4] = (uint8_t)(udp_len >> 8);
    udph[5] = (uint8_t)(udp_len & 0xFFU);
    udph[6] = 0x00U; udph[7] = 0x00U;   /* checksum (optional) */

    memcpy(udph + 8, data, payload_len);

    HAL_ETH_TransmitFrame(&heth, frame_len);
}

static void send_text(const char *str)
{
    eth_send_udp(str, (uint16_t)strlen(str));
}

static void send_feedback(uint32_t can_id, float q, float q_dot)
{
    char msg[80];
    const int len = snprintf(msg, sizeof(msg), "FBK 0x%02lX %.6f %.6f\r\n",
                             (unsigned long)can_id, (double)q, (double)q_dot);
    if (len > 0) {
        eth_send_udp(msg, (uint16_t)len);
    }
}

static void service_can_rx(void)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            return;
        }

        if (rx_header.IDE == CAN_ID_EXT) {
            const uint32_t ext_id    = rx_header.ExtId;
            const uint8_t  comm_type = (uint8_t)((ext_id >> 24) & 0x3FU);
            const uint8_t  node_id   = (uint8_t)((ext_id >> 8)  & 0xFFU);

            if (comm_type == RS04_COMM_MOTOR_FEEDBACK && rx_header.DLC >= 8U) {
                const uint16_t p_int  = ((uint16_t)rx_data[0] << 8) | rx_data[1];
                const uint16_t v_int  = ((uint16_t)rx_data[2] << 8) | rx_data[3];
                const float    q      = uint_to_float(p_int, P_MIN, P_MAX, 16);
                const float    q_dot  = uint_to_float(v_int, V_MIN, V_MAX, 16);
                send_feedback(node_id, q, q_dot);
            } else if (comm_type == RS04_COMM_GET_ID && (ext_id & 0xFFU) == 0xFEU) {
                char id_msg[32];
                motor_can_id = node_id;
                const int len = snprintf(id_msg, sizeof(id_msg), "ID 0x%02X\r\n", node_id);
                if (len > 0) {
                    send_text(id_msg);
                }
            }
        }
    }
}

static void process_line(const char *line)
{
    float q, kp, kd, tau;
    unsigned long can_id_ul = 0UL;

    if (sscanf(line, "CMD 0x%lx %f %f %f %f", &can_id_ul, &q, &kp, &kd, &tau) == 5) {
        motor_can_id = (uint32_t)(can_id_ul & 0xFFUL);
        send_motor_command(motor_can_id, q, 0.0f, kp, kd, tau);
        return;
    }

    if (sscanf(line, "CMD %f %f %f %f", &q, &kp, &kd, &tau) == 4) {
        send_motor_command(motor_can_id, q, 0.0f, kp, kd, tau);
        return;
    }

    if (strcmp(line, "ID?") == 0) {
        static const uint8_t id_cmd[8] = {0};
        (void)can_send_frame((RS04_COMM_GET_ID << 24) | (RS04_MASTER_CAN_ID << 8) | (motor_can_id & 0xFFU), id_cmd, 8U);
        return;
    }

    if (strcmp(line, "ZERO") == 0) {
        const uint8_t zero_cmd[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        (void)can_send_frame((RS04_COMM_SET_POS_ZERO << 24) | (RS04_MASTER_CAN_ID << 8) | (motor_can_id & 0xFFU), zero_cmd, 8U);
        return;
    }

    if (strcmp(line, "STOP") == 0) {
        const uint8_t stop_cmd[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        (void)can_send_frame((RS04_COMM_MOTOR_STOP << 24) | (RS04_MASTER_CAN_ID << 8) | (motor_can_id & 0xFFU), stop_cmd, 8U);
        motor_mode_enabled = false;
        return;
    }

    send_text("ERR unsupported command\r\n");
}

static bool can_send_frame(uint32_t ext_id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0U;

    tx_header.StdId = 0U;
    tx_header.ExtId = ext_id & 0x1FFFFFFFU;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)data, &tx_mailbox) != HAL_OK) {
        return false;
    }

    for (uint32_t timeout = 0U; timeout < 100000U; ++timeout) {
        if (!HAL_CAN_IsTxMessagePending(&hcan1, tx_mailbox)) {
            return true;
        }
    }

    HAL_CAN_AbortTxRequest(&hcan1, tx_mailbox);
    return false;
}

static void send_motor_command(uint32_t can_id, float q, float qd, float kp, float kd, float tau)
{
    const uint8_t enable_cmd[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    const uint8_t mode_cmd[8] = {
        (uint8_t)(RS04_RUN_MODE_INDEX & 0xFFU),
        (uint8_t)(RS04_RUN_MODE_INDEX >> 8),
        0x00, 0x00,
        RS04_MOVE_CONTROL_MODE,
        0x00, 0x00, 0x00
    };
    uint8_t payload[8];
    char err_msg[96];

    q = clampf_local(q, P_MIN, P_MAX);
    qd = clampf_local(qd, V_MIN, V_MAX);
    kp = clampf_local(kp, KP_MIN, KP_MAX);
    kd = clampf_local(kd, KD_MIN, KD_MAX);
    tau = clampf_local(tau, T_MIN, T_MAX);

    if (!motor_mode_enabled) {
        if (!can_send_frame((RS04_COMM_SET_SINGLE_PARAM << 24) | (RS04_MASTER_CAN_ID << 8) | (can_id & 0xFFU), mode_cmd, 8U)) {
            send_text("ERR RS04 mode set failed\r\n");
            return;
        }
        if (!can_send_frame((RS04_COMM_MOTOR_ENABLE << 24) | (RS04_MASTER_CAN_ID << 8) | (can_id & 0xFFU), enable_cmd, 8U)) {
            const int len = snprintf(err_msg, sizeof(err_msg), "ERR CAN enable failed id=0x%02lX esr=0x%08lX\r\n", (unsigned long)can_id, (unsigned long)hcan1.Instance->ESR);
            if (len > 0) {
                send_text(err_msg);
            }
            return;
        }
        motor_mode_enabled = true;
        for (volatile uint32_t i = 0U; i < 32000U; ++i) {
            __asm("NOP");
        }
    }

    {
        const uint32_t p_int = float_to_uint(q, P_MIN, P_MAX, 16);
        const uint32_t v_int = float_to_uint(qd, V_MIN, V_MAX, 16);
        const uint32_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 16);
        const uint32_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 16);
        const uint32_t t_int = float_to_uint(tau, T_MIN, T_MAX, 16);
        const uint32_t ext_id = (RS04_COMM_MOTION_CONTROL << 24) | ((t_int & 0xFFFFU) << 8) | (can_id & 0xFFU);

        payload[0] = (uint8_t)(p_int >> 8);
        payload[1] = (uint8_t)(p_int & 0xFFU);
        payload[2] = (uint8_t)(v_int >> 8);
        payload[3] = (uint8_t)(v_int & 0xFFU);
        payload[4] = (uint8_t)(kp_int >> 8);
        payload[5] = (uint8_t)(kp_int & 0xFFU);
        payload[6] = (uint8_t)(kd_int >> 8);
        payload[7] = (uint8_t)(kd_int & 0xFFU);

        if (!can_send_frame(ext_id, payload, 8U)) {
            const int len = snprintf(err_msg, sizeof(err_msg), "ERR CAN TX failed id=0x%02lX esr=0x%08lX\r\n", (unsigned long)can_id, (unsigned long)hcan1.Instance->ESR);
            if (len > 0) {
                send_text(err_msg);
            }
        }
    }
}

static void SystemClock_Config(void)
{
    /*
     * Primary:  HSE bypass (8 MHz ST-Link MCO) → PLL → 180 MHz + overdrive
     *   PLLM=8, PLLN=360, PLLP=2 → SYSCLK=180 MHz; APB1=45 MHz
     *   CAN: Prescaler=3, BS1=10TQ, BS2=4TQ → 1 Mbps at 45 MHz APB1
     *
     * Fallback: HSI 16 MHz → PLL → 168 MHz
     *   PLLM=16, PLLN=336, PLLP=2 → SYSCLK=168 MHz; APB1=42 MHz
     *   CAN: Prescaler=3, BS1=10TQ, BS2=4TQ → ~934 kbps (within motor tolerance)
     */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Try HSE bypass first (8 MHz from ST-Link MCO on Nucleo-F429ZI) */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8U;
    RCC_OscInitStruct.PLL.PLLN       = 360U;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7U;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        /* HSE unavailable — fall back to HSI, 168 MHz */
        memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
        RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
        RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
        RCC_OscInitStruct.PLL.PLLM            = 16U;
        RCC_OscInitStruct.PLL.PLLN            = 336U;
        RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
        RCC_OscInitStruct.PLL.PLLQ            = 7U;
        HAL_RCC_OscConfig(&RCC_OscInitStruct);
    }

    HAL_PWREx_EnableOverDrive();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/*
 * MX_ETH_Init — configure the STM32F429ZI built-in Ethernet MAC
 * connected to the on-board LAN8742A PHY via RMII.
 *
 * RMII pins on Nucleo-F429ZI (fixed by board hardware):
 *   PA1  – REF_CLK   PA2  – MDIO    PA7  – CRS_DV
 *   PB13 – TXD1      PC1  – MDC     PC4  – RXD0
 *   PC5  – RXD1      PG2  – TXEN    PG13 – TXD0
 */
static void MX_ETH_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable all required GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;

    /* PA1, PA2, PA7 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PC1, PC4, PC5 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PG2 (ETH_TXEN on Nucleo-F429ZI), PG13 (ETH_TXD0) */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* Enable SYSCFG and select RMII mode */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL;

    /* Enable ETHERNET clocks */
    __HAL_RCC_ETHMAC_CLK_ENABLE();
    __HAL_RCC_ETHMACTX_CLK_ENABLE();
    __HAL_RCC_ETHMACRX_CLK_ENABLE();

    heth.Instance               = ETH;
    heth.Init.AutoNegotiation   = ETH_AUTONEGOTIATION_ENABLE;
    heth.Init.PhyAddress        = ETH_PHY_ADDR;
    heth.Init.MACAddr           = (uint8_t *)my_mac;
    heth.Init.RxMode            = ETH_RXPOLLING_MODE;
    heth.Init.ChecksumMode      = ETH_CHECKSUM_BY_SOFTWARE;
    heth.Init.MediaInterface    = ETH_MEDIA_INTERFACE_RMII;

    HAL_ETH_Init(&heth);

    HAL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
    HAL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

    HAL_ETH_Start(&heth);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* CAN1 on PD0 (RX) / PD1 (TX) — AF9.
         * Only configure one pin set. Driving both PB8/PB9 and PD0/PD1 as
         * CAN1 AF simultaneously corrupts the bus since both sets feed the
         * same CAN RX line. */
        GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}

static void MX_CAN1_Init(void)
{
    CAN_FilterTypeDef can_filter = {0};

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;  /* 1 Mbps at 180 MHz/APB1=45 MHz (tested) */
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = ENABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    HAL_CAN_Init(&hcan1);

    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
    HAL_CAN_Start(&hcan1);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}
