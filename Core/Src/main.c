/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay100(){
  for(int i = 0; i<1000; i++);
}

volatile uint8_t rdata[50] = {};
uint8_t zeros[50] = {};



bool send_deviate_expect(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, const uint8_t *expected_data, uint16_t expectedSize, uint32_t Timeout){
  memset(rdata, 0, 50);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
  asm volatile ("" : : "r" (*(unsigned int *) huart->Instance->SR));
  asm volatile ("" : : "r" (*(unsigned int *) huart->Instance->DR));
  HAL_UART_Receive_IT(&huart1, rdata, expectedSize);
  HAL_UART_Transmit_IT(huart, pData, Size);
  for(int i = 0; i<Timeout; i++);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
  return !memcmp(rdata, expected_data, expectedSize);
}
bool send_thing(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout){
  send_deviate_expect(huart,pData, Size, pData, Size, Timeout);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t cnt = 0;
  uint8_t adress99[167] = {0x1, 0x0, 0,  0,  0,  0x6, 0xA0,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
  };
  while (1)
  {
    uint8_t pane_addresses[] = {0x03, 0x07};
    int rx_ok = 1;
    uint8_t address1[] = {0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01};
    uint8_t address111[] = {0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x09};
    if(!send_deviate_expect(&huart1, address1, sizeof(address1), address111, sizeof(address111), 20000)) {
      HAL_Delay(100);
      continue;
    }
    delay100();

    uint8_t address2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0E, 0x01, 0xC8};
    if(!send_thing(&huart1, address2, sizeof(address2), 1000)) continue;
    delay100();
    uint8_t address4[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00};
    if(!send_thing(&huart1, address4, sizeof(address4), 1000)) continue;
    delay100();
    uint8_t address3[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0x00};
    if(!send_thing(&huart1, address3, sizeof(address3), 1000)){
      continue;
    }

    HAL_Delay(5);

    uint8_t address9[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x30, 0x02, 0x00, 0x00};
    if(!send_thing(&huart1, address9, sizeof(address9), 1000)) continue;

    HAL_Delay(5);
    uint8_t address5[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00};
    if(!send_thing(&huart1, address5, sizeof(address5), 1000)) continue;

    HAL_Delay(5);
    uint8_t address6[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0A, 0x01, 0x03};
    if(!send_thing(&huart1, address6, sizeof(address6), 1000)) continue;

    HAL_Delay(3);
    uint8_t address7[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x16, 0x01, 0x01};
    if(!send_thing(&huart1, address7, sizeof(address7), 1000)) continue;

    HAL_Delay(14);
    uint8_t address8[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x30, 0x02, 0x00, 0x01};
    if(!send_thing(&huart1, address8, sizeof(address8), 1000)) continue;
    HAL_Delay(1);

    uint8_t addresses[]  = {0x6, 0x9, 0xC, 0xE, 0x10, 0x12, 0x14, 0x16, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22};

    for(int addr = 0; addr < sizeof(addresses); addr++){
      if(!send_thing(&huart1, address3, sizeof(address3), 1000)){
        rx_ok = 0;
        break;
      }
      address6[7] = addresses[addr];
      delay100();
      if(!send_thing(&huart1, address6, sizeof(address6), 1000)){
        rx_ok = 0;
        break;
      }
      delay100();
      if(!send_thing(&huart1, address5, sizeof(address5), 1000)){
        rx_ok = 0;
        break;
      }
      delay100();
      delay100();
      delay100();
      delay100();
      delay100();
      if(!send_thing(&huart1, address7, sizeof(address7), 1000)){
        rx_ok = 0;
        break;
      }
      HAL_Delay(25);
    }
    if(!rx_ok){
      continue;
    }


    while(1){
      if(!send_thing(&huart1, address3, sizeof(address3), 1000)){
        rx_ok = 0;
        break;
      }
      delay100();
      if(!send_thing(&huart1, address6, sizeof(address6), 1000)){
        rx_ok = 0;
        break;
      }
      delay100();
      if(!send_thing(&huart1, address5, sizeof(address5), 1000)){
        rx_ok = 0;
        break;
      }
      delay100();
      delay100();
      delay100();
      delay100();
      delay100();
      if(!send_thing(&huart1, address7, sizeof(address7), 1000)){
        rx_ok = 0;
        break;
      }


      adress99[(cnt%160)+7] = cnt/160 ? 0xFF : 0x0;
      HAL_UART_Transmit_IT(&huart1, adress99, sizeof(adress99));
      cnt = (cnt+1)%320;

      HAL_Delay(25);
    }

//    while(1){
//
//      for(int pane = 0; pane < sizeof(pane_addresses); pane++){
//
//
//        HAL_Delay(1);
//      }
//    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
