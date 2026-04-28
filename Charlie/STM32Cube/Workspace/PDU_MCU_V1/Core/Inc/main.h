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
#include "stm32g4xx_hal.h"

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
#define MCU_HB_Pin GPIO_PIN_1
#define MCU_HB_GPIO_Port GPIOC
#define CMD_FAULT_Pin GPIO_PIN_2
#define CMD_FAULT_GPIO_Port GPIOC
#define CMD_ARM_Pin GPIO_PIN_3
#define CMD_ARM_GPIO_Port GPIOC
#define FPGA_SPARE_OUT_0_Pin GPIO_PIN_0
#define FPGA_SPARE_OUT_0_GPIO_Port GPIOA
#define FPGA_SPARE_OUT_1_Pin GPIO_PIN_1
#define FPGA_SPARE_OUT_1_GPIO_Port GPIOA
#define FAULT_LATCH_Pin GPIO_PIN_2
#define FAULT_LATCH_GPIO_Port GPIOA
#define SPARE_ADC_0_Pin GPIO_PIN_4
#define SPARE_ADC_0_GPIO_Port GPIOA
#define SPARE_ADC_1_Pin GPIO_PIN_5
#define SPARE_ADC_1_GPIO_Port GPIOA
#define THERM_0_Pin GPIO_PIN_6
#define THERM_0_GPIO_Port GPIOA
#define THERM_1_Pin GPIO_PIN_7
#define THERM_1_GPIO_Port GPIOA
#define THERM_2_Pin GPIO_PIN_4
#define THERM_2_GPIO_Port GPIOC
#define THERM_3_Pin GPIO_PIN_5
#define THERM_3_GPIO_Port GPIOC
#define V_SOURCE_Pin GPIO_PIN_0
#define V_SOURCE_GPIO_Port GPIOB
#define V_BUS_Pin GPIO_PIN_1
#define V_BUS_GPIO_Port GPIOB
#define I_COIL_Pin GPIO_PIN_2
#define I_COIL_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOB
#define SW_FAULT_Pin GPIO_PIN_12
#define SW_FAULT_GPIO_Port GPIOB
#define PGOOD_SW_Pin GPIO_PIN_13
#define PGOOD_SW_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_14
#define RS485_DE_GPIO_Port GPIOB
#define PGOOD_24V_Pin GPIO_PIN_15
#define PGOOD_24V_GPIO_Port GPIOB
#define AUX_SW_0_Pin GPIO_PIN_8
#define AUX_SW_0_GPIO_Port GPIOC
#define AUX_SW_1_Pin GPIO_PIN_9
#define AUX_SW_1_GPIO_Port GPIOC
#define AUX_SW_2_Pin GPIO_PIN_8
#define AUX_SW_2_GPIO_Port GPIOA
#define AUX_SW_3_Pin GPIO_PIN_9
#define AUX_SW_3_GPIO_Port GPIOA
#define CAN_STB_Pin GPIO_PIN_10
#define CAN_STB_GPIO_Port GPIOA
#define LED_0_Pin GPIO_PIN_4
#define LED_0_GPIO_Port GPIOB
#define BUZZER_P_Pin GPIO_PIN_5
#define BUZZER_P_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_6
#define LED_1_GPIO_Port GPIOB
#define BUZZER_N_Pin GPIO_PIN_7
#define BUZZER_N_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
