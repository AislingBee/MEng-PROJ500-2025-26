/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU0_SCK_Pin GPIO_PIN_2
#define IMU0_SCK_GPIO_Port GPIOE
#define IMU0_INT1_Pin GPIO_PIN_3
#define IMU0_INT1_GPIO_Port GPIOE
#define IMU0_INT1_EXTI_IRQn EXTI3_IRQn
#define IMU0_INT2_Pin GPIO_PIN_4
#define IMU0_INT2_GPIO_Port GPIOE
#define IMU0_INT2_EXTI_IRQn EXTI4_IRQn
#define IMU0_MISO_Pin GPIO_PIN_5
#define IMU0_MISO_GPIO_Port GPIOE
#define IMU0_MOSI_Pin GPIO_PIN_6
#define IMU0_MOSI_GPIO_Port GPIOE
#define IMU0_NCS_Pin GPIO_PIN_13
#define IMU0_NCS_GPIO_Port GPIOC
#define LED_0_Pin GPIO_PIN_0
#define LED_0_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_2
#define LED_1_GPIO_Port GPIOC
#define MCU_HB_Pin GPIO_PIN_3
#define MCU_HB_GPIO_Port GPIOC
#define THERMISTOR_Pin GPIO_PIN_0
#define THERMISTOR_GPIO_Port GPIOA
#define BUZZER_N_Pin GPIO_PIN_8
#define BUZZER_N_GPIO_Port GPIOE
#define BUZZER_P_Pin GPIO_PIN_9
#define BUZZER_P_GPIO_Port GPIOE
#define ETH_NRST_Pin GPIO_PIN_10
#define ETH_NRST_GPIO_Port GPIOB
#define CAN_MTR_L_RX_Pin GPIO_PIN_12
#define CAN_MTR_L_RX_GPIO_Port GPIOD
#define CAN_MTR_L_TX_Pin GPIO_PIN_13
#define CAN_MTR_L_TX_GPIO_Port GPIOD
#define CAN_MTR_L_STB_Pin GPIO_PIN_14
#define CAN_MTR_L_STB_GPIO_Port GPIOD
#define EXP_I2C_SDA_Pin GPIO_PIN_9
#define EXP_I2C_SDA_GPIO_Port GPIOC
#define EXP_I2C_SCL_Pin GPIO_PIN_8
#define EXP_I2C_SCL_GPIO_Port GPIOA
#define IMU1_INT1_Pin GPIO_PIN_9
#define IMU1_INT1_GPIO_Port GPIOA
#define IMU1_INT1_EXTI_IRQn EXTI9_5_IRQn
#define CAN_MTR_R_STB_Pin GPIO_PIN_10
#define CAN_MTR_R_STB_GPIO_Port GPIOA
#define CAN_MTR_R_RX_Pin GPIO_PIN_11
#define CAN_MTR_R_RX_GPIO_Port GPIOA
#define CAN_MTR_R_TX_Pin GPIO_PIN_12
#define CAN_MTR_R_TX_GPIO_Port GPIOA
#define DBG_SWDIO_Pin GPIO_PIN_13
#define DBG_SWDIO_GPIO_Port GPIOA
#define DBG_SWCLK_Pin GPIO_PIN_14
#define DBG_SWCLK_GPIO_Port GPIOA
#define IMU1_NCS_Pin GPIO_PIN_15
#define IMU1_NCS_GPIO_Port GPIOA
#define IMU1_SCK_Pin GPIO_PIN_10
#define IMU1_SCK_GPIO_Port GPIOC
#define IMU1_MISO_Pin GPIO_PIN_11
#define IMU1_MISO_GPIO_Port GPIOC
#define IMU1_MOSI_Pin GPIO_PIN_12
#define IMU1_MOSI_GPIO_Port GPIOC
#define EXP_UART_RX_Pin GPIO_PIN_0
#define EXP_UART_RX_GPIO_Port GPIOD
#define EXP_UART_TX_Pin GPIO_PIN_1
#define EXP_UART_TX_GPIO_Port GPIOD
#define EXP_GPIO0_Pin GPIO_PIN_2
#define EXP_GPIO0_GPIO_Port GPIOD
#define EXP_GPIO1_Pin GPIO_PIN_3
#define EXP_GPIO1_GPIO_Port GPIOD
#define EXP_GPIO2_Pin GPIO_PIN_4
#define EXP_GPIO2_GPIO_Port GPIOD
#define UART_DBG_TX_Pin GPIO_PIN_5
#define UART_DBG_TX_GPIO_Port GPIOD
#define UART_DBG_RX_Pin GPIO_PIN_6
#define UART_DBG_RX_GPIO_Port GPIOD
#define EXP_GPIO3_Pin GPIO_PIN_7
#define EXP_GPIO3_GPIO_Port GPIOD
#define DBG_SWO_Pin GPIO_PIN_3
#define DBG_SWO_GPIO_Port GPIOB
#define CAN_PDU_STB_Pin GPIO_PIN_4
#define CAN_PDU_STB_GPIO_Port GPIOB
#define CAN_PDU_RX_Pin GPIO_PIN_5
#define CAN_PDU_RX_GPIO_Port GPIOB
#define CAN_PDU_TX_Pin GPIO_PIN_6
#define CAN_PDU_TX_GPIO_Port GPIOB
#define PDU_FAULT_Pin GPIO_PIN_7
#define PDU_FAULT_GPIO_Port GPIOB
#define PDU_FAULT_EXTI_IRQn EXTI9_5_IRQn
#define ESP_STATUS_Pin GPIO_PIN_8
#define ESP_STATUS_GPIO_Port GPIOB
#define ESP_STATUS_EXTI_IRQn EXTI9_5_IRQn
#define ESP_RST_Pin GPIO_PIN_9
#define ESP_RST_GPIO_Port GPIOB
#define UART_ESP_RX_Pin GPIO_PIN_0
#define UART_ESP_RX_GPIO_Port GPIOE
#define UART_ESP_TX_Pin GPIO_PIN_1
#define UART_ESP_TX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
