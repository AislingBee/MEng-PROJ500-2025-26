/*
 * main.c
 * STM32 firmware bridge for ROS <-> Robostride motor via CAN.
 *
 * This example receives ASCII motor commands on UART from a host PC,
 * packs them into a CAN frame, and sends them to the motor.
 * It also receives CAN feedback frames from the motor and forwards them
 * back to the host over UART.
 *
 * Host protocol examples:
 *   CMD 1.0 20.0 1.0 0.0\n      -> send q=1.0, kp=20.0, kd=1.0, tau=0.0
 *   FBK 0x201 0.1234 -0.0056\n  <- received CAN feedback with q and q_dot
 *
 * The ROS side can implement a small serial bridge that converts commands
 * into this ASCII protocol and publishes received feedback into the
 * existing /motor_can or /motor_feedback topics.
 */

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

#define UART_RX_BUF_SIZE 128
static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static uint16_t uart_rx_len = 0;
static uint8_t uart_rx_byte;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void process_uart_line(const char *line);
static void send_motor_command(float q, float kp, float kd, float tau);
static void send_uart_feedback(uint32_t can_id, float q, float q_dot);
static void float_to_bytes(float value, uint8_t bytes[4]);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_CAN1_Init();

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        // CAN start error
        while (1);
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        while (1);
    }

    const char *boot_msg = "STM32 CAN<->UART bridge ready\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)boot_msg, strlen(boot_msg), HAL_MAX_DELAY);

    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

    while (1) {
        // Blink the on-board LED to show the firmware is running.
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500);
    }
}

static void process_uart_line(const char *line)
{
    float q, kp, kd, tau;
    uint32_t can_id;

    if (sscanf(line, "CMD %f %f %f %f", &q, &kp, &kd, &tau) == 4) {
        send_motor_command(q, kp, kd, tau);
        return;
    }

    if (sscanf(line, "FBK 0x%X %f %f", &can_id, &q, &tau) == 3) {
        // Received a feedback request or debug message from the host.
        // Not used in this bridge implementation.
        return;
    }

    const char *err = "ERR unsupported command\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)err, strlen(err), HAL_MAX_DELAY);
}

static void send_motor_command(float q, float kp, float kd, float tau)
{
    uint8_t payload[8];
    float_to_bytes(q, &payload[0]);
    float_to_bytes(kp, &payload[4]);

    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId = 0x201;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, payload, &tx_mailbox) != HAL_OK) {
        const char *err = "ERR CAN TX failed\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)err, strlen(err), HAL_MAX_DELAY);
    } else {
        const char msg[] = "CMD_SENT\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
    }
}

static void send_uart_feedback(uint32_t can_id, float q, float q_dot)
{
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "FBK 0x%03lX %.6f %.6f\r\n", (unsigned long)can_id, q, q_dot);
    if (len > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, HAL_MAX_DELAY);
    }
}

static void float_to_bytes(float value, uint8_t bytes[4])
{
    uint8_t *p = (uint8_t *)&value;
    bytes[0] = p[0];
    bytes[1] = p[1];
    bytes[2] = p[2];
    bytes[3] = p[3];
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        if (uart_rx_len < UART_RX_BUF_SIZE - 1) {
            uart_rx_buf[uart_rx_len++] = uart_rx_byte;
            if (uart_rx_byte == '\n' || uart_rx_byte == '\r') {
                uart_rx_buf[uart_rx_len] = '\0';
                process_uart_line((char *)uart_rx_buf);
                uart_rx_len = 0;
            }
        } else {
            uart_rx_len = 0;
        }
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        if (rx_header.DLC >= 8) {
            float q, q_dot;
            memcpy(&q, &rx_data[0], sizeof(float));
            memcpy(&q_dot, &rx_data[4], sizeof(float));
            send_uart_feedback(rx_header.StdId, q, q_dot);
        }
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void MX_CAN1_Init(void)
{
    CAN_FilterTypeDef can_filter = {0};

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 16;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
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
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure PA5 as output for the on-board LED */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Start with the LED off */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}
