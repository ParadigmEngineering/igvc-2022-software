/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bldc_interface.h"
#include "can.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_handler.h"
#include "bldc_interface_uart.h"
#include "can_message_defs.h"
#include "state.h"
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
BldcInterface motor1 = {0};
BldcInterface motor2 = {0};
BldcInterface motor3 = {0};
state curr_state = BOOT;
state next_state = BOOT;
uint32_t last_heartbeat_received = 0;
static const uint32_t HEARTBEAT_EXPIRED_MS = 1000;
uint8_t vesc_data_valid[3] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void write_packet_motor1(unsigned char* data, unsigned int len)
{
  HAL_UART_Transmit(&huart2, data, len, 1000);
}

static void write_packet_motor2(unsigned char* data, unsigned int len)
{
  HAL_UART_Transmit(&huart3, data, len, 1000);
}

static void write_packet_motor3(unsigned char* data, unsigned int len)
{
  HAL_UART_Transmit(&huart1, data, len, 1000);
}

uint8_t rx_data_motor1;
uint8_t rx_data_motor2;
uint8_t rx_data_motor3;

uint32_t last_time;
uint32_t second_last_time;

// E-stop interrupts
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  if (gpio_pin == GPIO_PIN_7 || gpio_pin == GPIO_PIN_4)
  {
    next_state = STANDBY;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BldcInterface* motor;
  unsigned char* data;
  if (huart == &huart1)
  {
    motor = &motor3;
    data = &rx_data_motor3;
  }
  else if (huart == &huart2)
  {
    motor = &motor1;
    data = &rx_data_motor1;
  }
  else // huart == &huart3
  {
    motor = &motor2;
    data = &rx_data_motor2;
  }

  bldc_interface_uart_process_byte(motor, *data);
  HAL_UART_Receive_IT(huart, data, 1);
}

// Ask Dan if this is valid to turn off
void turn_off_lamps(void)
{
  HAL_GPIO_WritePin(LAMP1_ON_GPIO_Port, LAMP1_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LAMP2_ON_GPIO_Port, LAMP2_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LAMP3_ON_GPIO_Port, LAMP3_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LAMP4_ON_GPIO_Port, LAMP4_ON_Pin, GPIO_PIN_RESET);
}

static void bldc_values_received_motor1(mc_values* val)
{
  static const uint32_t motor1_ids[] = {
    0x40,
    0x41,
    0x42,
    0x43,
    0x44
  };

  vesc_data_valid[0] = 1;
  
  CanMessage message1;
  message1.id = motor1_ids[0];
  message1.len = 8;
  memcpy(message1.data, &val->v_in, 4);
  memcpy((message1.data + 4), &val->temp_mos, 4);

  CanMessage message2;
  message2.id = motor1_ids[1];
  message2.len = 8;
  memcpy(message2.data, &val->temp_motor, 4);
  memcpy((message2.data + 4), &val->current_motor, 4);

  CanMessage message3;
  message3.id = motor1_ids[2];
  message3.len = 8;
  memcpy(message3.data, &val->current_in, 4);
  memcpy((message3.data + 4), &val->id, 4);
  
  CanMessage message4;
  message4.id = motor1_ids[3];
  message4.len = 8;
  memcpy(message4.data, &val->iq, 4);
  memcpy((message4.data + 4), &val->rpm, 4);

  CanMessage message5;
  message5.id = motor1_ids[4];
  message5.len = 8;
  memcpy(message5.data, &val->duty_now, 4);
  memcpy((message5.data + 4), &val->amp_hours, 4);

  send_can_message_blocking(&message1);
  send_can_message_blocking(&message2);
  send_can_message_blocking(&message3);
  send_can_message_blocking(&message4);
  send_can_message_blocking(&message5);
}

static void bldc_values_received_motor2(mc_values* val)
{
  static const uint32_t motor2_ids[] = {
    0x50,
    0x51,
    0x52,
    0x53,
    0x54
  };
  
  vesc_data_valid[1] = 1;

  CanMessage message1;
  message1.id = motor2_ids[0];
  message1.len = 8;
  memcpy(message1.data, &val->v_in, 4);
  memcpy((message1.data + 4), &val->temp_mos, 4);

  CanMessage message2;
  message2.id = motor2_ids[1];
  message2.len = 8;
  memcpy(message2.data, &val->temp_motor, 4);
  memcpy((message2.data + 4), &val->current_motor, 4);

  CanMessage message3;
  message3.id = motor2_ids[2];
  message3.len = 8;
  memcpy(message3.data, &val->current_in, 4);
  memcpy((message3.data + 4), &val->id, 4);
  
  CanMessage message4;
  message4.id = motor2_ids[3];
  message4.len = 8;
  memcpy(message4.data, &val->iq, 4);
  memcpy((message4.data + 4), &val->rpm, 4);

  CanMessage message5;
  message5.id = motor2_ids[4];
  message5.len = 8;
  memcpy(message5.data, &val->duty_now, 4);
  memcpy((message5.data + 4), &val->amp_hours, 4);

  send_can_message_blocking(&message1);
  send_can_message_blocking(&message2);
  send_can_message_blocking(&message3);
  send_can_message_blocking(&message4);
  send_can_message_blocking(&message5);
}
static void bldc_values_received_motor3(mc_values* val)
{
  static const uint32_t motor3_ids[] = {
    0x60,
    0x61,
    0x62,
    0x63,
    0x64
  };
  
  vesc_data_valid[2] = 1;

  CanMessage message1;
  message1.id = motor3_ids[0];
  message1.len = 8;
  memcpy(message1.data, &val->v_in, 4);
  memcpy((message1.data + 4), &val->temp_mos, 4);

  CanMessage message2;
  message2.id = motor3_ids[1];
  message2.len = 8;
  memcpy(message2.data, &val->temp_motor, 4);
  memcpy((message2.data + 4), &val->current_motor, 4);

  CanMessage message3;
  message3.id = motor3_ids[2];
  message3.len = 8;
  memcpy(message3.data, &val->current_in, 4);
  memcpy((message3.data + 4), &val->id, 4);
  
  CanMessage message4;
  message4.id = motor3_ids[3];
  message4.len = 8;
  memcpy(message4.data, &val->iq, 4);
  memcpy((message4.data + 4), &val->rpm, 4);

  CanMessage message5;
  message5.id = motor3_ids[4];
  message5.len = 8;
  memcpy(message5.data, &val->duty_now, 4);
  memcpy((message5.data + 4), &val->amp_hours, 4);

  send_can_message_blocking(&message1);
  send_can_message_blocking(&message2);
  send_can_message_blocking(&message3);
  send_can_message_blocking(&message4);
  send_can_message_blocking(&message5);
}

// Must receive ack bit
void diagnose_node()
{
  bool isGood = true;
  //GPIO_PinState isBattGood = GPIO_PIN_SET;

  // Todo: Vescs, BATT_GOOD (Frank said Batt Good GPIO isnt working, comment out for now)
  bldc_interface_get_values(&motor1);
  HAL_Delay(100);
  bldc_interface_get_values(&motor2);
  HAL_Delay(100);
  bldc_interface_get_values(&motor3);
  HAL_Delay(100);

  //isBattGood = HAL_GPIO_ReadPin(BATT_GOOD_GPIO_Port, BATT_GOOD_Pin);

  // if (motor1.getSomething != Good || motor2.getSomething != Good || motor3.getSomethign != Good)
  // {
  //     isGood = false;
  // }

  // if (isBattGood == GPIO_PIN_RESET)
  // {
  //   isGood = false;
  // }

  if (isGood)
  {
    CanMessage goodMessage;
    goodMessage.id = NODE_GOOD_CAN_ID_MASK;
    goodMessage.len = 0;
    send_can_message_blocking(&goodMessage);
  }

  else
  {
    CanMessage badMessage;
    badMessage.id = NODE_BAD_CAN_ID_MASK;
    badMessage.len = 0;
    send_can_message_blocking(&badMessage);
  }
}

static void write_lamps(void)
{
  // Solid LEDs
  if (curr_state == STANDBY)
  {
    HAL_GPIO_WritePin(LAMP1_ON_GPIO_Port, LAMP1_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP2_ON_GPIO_Port, LAMP2_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP3_ON_GPIO_Port, LAMP3_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP4_ON_GPIO_Port, LAMP4_ON_Pin, GPIO_PIN_SET);
  }

  // Blink LEDs in Autonomous drive
  else if (curr_state == AUTONOMOUS)
  {
    // Wait 0.5 seconds for flash in Autonomous
    if (HAL_GetTick()-last_time > 250)
    {
      HAL_GPIO_TogglePin(LAMP1_ON_GPIO_Port, LAMP1_ON_Pin);
      HAL_GPIO_TogglePin(LAMP2_ON_GPIO_Port, LAMP2_ON_Pin);
      HAL_GPIO_TogglePin(LAMP3_ON_GPIO_Port, LAMP3_ON_Pin);
      HAL_GPIO_TogglePin(LAMP4_ON_GPIO_Port, LAMP4_ON_Pin);
      last_time = HAL_GetTick();
    }
  }

  // Blink LEDs in Manual drive
  else if (curr_state == MANUAL)
  {
    // Wait 0.3 seconds for flash in Manual
    if (HAL_GetTick()-second_last_time > 2000)
    {
      HAL_GPIO_TogglePin(LAMP1_ON_GPIO_Port, LAMP1_ON_Pin);
      HAL_GPIO_TogglePin(LAMP2_ON_GPIO_Port, LAMP2_ON_Pin);
      HAL_GPIO_TogglePin(LAMP3_ON_GPIO_Port, LAMP3_ON_Pin);
      HAL_GPIO_TogglePin(LAMP4_ON_GPIO_Port, LAMP4_ON_Pin);
      second_last_time = HAL_GetTick();
    }
  }

  else
  {
    HAL_GPIO_WritePin(LAMP1_ON_GPIO_Port, LAMP1_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP2_ON_GPIO_Port, LAMP2_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP3_ON_GPIO_Port, LAMP3_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAMP4_ON_GPIO_Port, LAMP4_ON_Pin, GPIO_PIN_SET);
  }
}

static int heartbeat_expired(uint32_t last_heartbeat_received_ms)
{
  return (HAL_GetTick() - last_heartbeat_received_ms) > HEARTBEAT_EXPIRED_MS;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  bldc_interface_uart_init(&motor1, write_packet_motor1);
  bldc_interface_set_rx_value_func(&motor1, bldc_values_received_motor1);

  bldc_interface_uart_init(&motor2, write_packet_motor2);
  bldc_interface_set_rx_value_func(&motor2, bldc_values_received_motor2);

  bldc_interface_uart_init(&motor3, write_packet_motor3);
  bldc_interface_set_rx_value_func(&motor3, bldc_values_received_motor3);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  last_time = HAL_GetTick();
  second_last_time = HAL_GetTick();

  CAN_FilterTypeDef sf;
  sf.FilterMaskIdHigh = 0x0000;
  sf.FilterMaskIdLow = 0x0000;
  sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sf.FilterBank = 0;
  sf.FilterMode = CAN_FILTERMODE_IDMASK;
  sf.FilterScale = CAN_FILTERSCALE_32BIT;
  sf.FilterActivation = CAN_FILTER_ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan, &sf) != HAL_OK) {
    Error_Handler();
  }
  HAL_CAN_Start(&hcan);

  HAL_UART_Receive_IT(&huart1, &rx_data_motor1, 1);
  HAL_UART_Receive_IT(&huart2, &rx_data_motor2, 1);
  HAL_UART_Receive_IT(&huart3, &rx_data_motor3, 1);

  //HAL_I2C_Master_Receive_IT(&hi2c1, hi2c1.Init.OwnAddress1, hi2c1.pBuffPtr, hi2c1.XferSize);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    handle_can_messages(5);

    if (heartbeat_expired(last_heartbeat_received))
    {
      curr_state = BOOT;
      next_state = BOOT;
      vesc_data_valid[0] = 0;
      vesc_data_valid[1] = 0;
      vesc_data_valid[2] = 0;
      last_heartbeat_received = 0;
      continue;
    }
    if (curr_state == MANUAL)
    {
      bldc_interface_set_rpm(&motor3, 200);
    }
    HAL_Delay(100);
    bldc_interface_get_values(&motor1);
    HAL_Delay(100);
    bldc_interface_get_values(&motor2);
    HAL_Delay(100);
    bldc_interface_get_values(&motor3);
    HAL_Delay(100);

    // Actuate GPIOs based on current state
    write_lamps();
    curr_state = next_state;  // next state is set based on CAN Messages

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
