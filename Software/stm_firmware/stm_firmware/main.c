/*
 * main.c
 * STM32 serial-to-CAN bridge for RobStride RS04 motor control.
 *
 * Serial protocol from the host:
 *   CMD 0xNN <q> <kp> <kd> <tau>\n
 *
 * Feedback back to the host:
 *   FBK 0xNN <q> <q_dot>\n
 */

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan1;

#define UART_RX_BUF_SIZE 128U
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

static char uart_rx_buf[UART_RX_BUF_SIZE];
static uint16_t uart_rx_len = 0U;
static bool motor_mode_enabled = false;
static uint32_t motor_can_id = DEFAULT_MOTOR_CAN_ID;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
static void process_uart_line(const char *line);
static void send_motor_command(uint32_t can_id, float q, float qd, float kp, float kd, float tau);
static void send_uart_feedback(uint32_t can_id, float q, float q_dot);
static void service_uart_rx(void);
static void service_can_rx(void);
static bool can_send_frame(uint32_t ext_id, const uint8_t *data, uint8_t dlc);

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

static void uart_send_char(char c)
{
    while ((USART3->SR & USART_SR_TXE) == 0U) {
    }
    USART3->DR = (uint8_t)c;
}

static void uart_send_string(const char *str)
{
    while (*str != '\0') {
        uart_send_char(*str++);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_CAN1_Init();

    while (1) {
        service_uart_rx();
        service_can_rx();
    }
}

static void service_uart_rx(void)
{
    while ((USART3->SR & USART_SR_RXNE) != 0U) {
        const char c = (char)(USART3->DR & 0xFFU);

        if (c == '\r' || c == '\n') {
            if (uart_rx_len > 0U) {
                uart_rx_buf[uart_rx_len] = '\0';
                process_uart_line(uart_rx_buf);
                uart_rx_len = 0U;
            }
            continue;
        }

        if (uart_rx_len < (UART_RX_BUF_SIZE - 1U)) {
            uart_rx_buf[uart_rx_len++] = c;
        } else {
            uart_rx_len = 0U;
        }
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
            const uint32_t ext_id = rx_header.ExtId;
            const uint8_t comm_type = (uint8_t)((ext_id >> 24) & 0x3FU);
            const uint8_t node_id = (uint8_t)((ext_id >> 8) & 0xFFU);

            if (comm_type == RS04_COMM_MOTOR_FEEDBACK && rx_header.DLC >= 8U) {
                const uint16_t p_int = ((uint16_t)rx_data[0] << 8) | rx_data[1];
                const uint16_t v_int = ((uint16_t)rx_data[2] << 8) | rx_data[3];
                const float q = uint_to_float(p_int, P_MIN, P_MAX, 16);
                const float q_dot = uint_to_float(v_int, V_MIN, V_MAX, 16);
                send_uart_feedback(node_id, q, q_dot);
            } else if (comm_type == RS04_COMM_GET_ID && (ext_id & 0xFFU) == 0xFEU) {
                char id_msg[32];
                motor_can_id = node_id;
                const int len = snprintf(id_msg, sizeof(id_msg), "ID 0x%02X\r\n", node_id);
                if (len > 0) {
                    uart_send_string(id_msg);
                }
            }
        }
    }
}

static void process_uart_line(const char *line)
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

    uart_send_string("ERR unsupported command\r\n");
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
            uart_send_string("ERR RS04 mode set failed\r\n");
            return;
        }
        if (!can_send_frame((RS04_COMM_MOTOR_ENABLE << 24) | (RS04_MASTER_CAN_ID << 8) | (can_id & 0xFFU), enable_cmd, 8U)) {
            const int len = snprintf(err_msg, sizeof(err_msg), "ERR CAN enable failed id=0x%02lX esr=0x%08lX\r\n", (unsigned long)can_id, (unsigned long)hcan1.Instance->ESR);
            if (len > 0) {
                uart_send_string(err_msg);
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
                uart_send_string(err_msg);
            }
        }
    }
}

static void send_uart_feedback(uint32_t can_id, float q, float q_dot)
{
    char msg[80];
    const int len = snprintf(
        msg,
        sizeof(msg),
        "FBK 0x%02lX %.6f %.6f\r\n",
        (unsigned long)can_id,
        (double)q,
        (double)q_dot
    );

    if (len > 0) {
        uart_send_string(msg);
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

    HAL_SYSTICK_Config(16000000U / 1000U);
}

static void MX_USART3_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    USART3->CR1 = 0U;
    USART3->BRR = 139U;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* Common CAN1 mappings on the Nucleo-F429ZI: PB8/PB9 and PD0/PD1. */
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;

        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}

static void MX_CAN1_Init(void)
{
    CAN_FilterTypeDef can_filter = {0};

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 2;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
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
