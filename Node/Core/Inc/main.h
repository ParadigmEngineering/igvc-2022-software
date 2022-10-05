/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#include "bldc_interface.h"


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern BldcInterface motor1;
extern BldcInterface motor2;
extern BldcInterface motor3;
extern uint8_t vesc_data_valid[3];
int heartbeat_expired(uint32_t last_heartbeat_received_ms);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LAMP3_ON_Pin GPIO_PIN_13
#define LAMP3_ON_GPIO_Port GPIOC
#define LAMP2_ON_Pin GPIO_PIN_14
#define LAMP2_ON_GPIO_Port GPIOC
#define LAMP1_ON_Pin GPIO_PIN_15
#define LAMP1_ON_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOF
#define ESC1_ON_Pin GPIO_PIN_0
#define ESC1_ON_GPIO_Port GPIOA
#define ESC1_ADC2_UNFILT_Pin GPIO_PIN_1
#define ESC1_ADC2_UNFILT_GPIO_Port GPIOA
#define ESC1_TX_Pin GPIO_PIN_2
#define ESC1_TX_GPIO_Port GPIOA
#define ESC1_RX_Pin GPIO_PIN_3
#define ESC1_RX_GPIO_Port GPIOA
#define BATT_GOOD_Pin GPIO_PIN_10
#define BATT_GOOD_GPIO_Port GPIOA
#define LAMP4_ON_Pin GPIO_PIN_9
#define LAMP4_ON_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
