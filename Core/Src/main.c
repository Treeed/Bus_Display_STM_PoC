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
unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
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
  uint8_t images[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,127,255,248,0,119,255,255,255,255,240,0,56,0,0,255,64,0,15,255,252,0,0,255,128,24,252,110,0,255,128,64,0,0,0,0,1,255,0,127,8,135,35,255,128,0,68,240,31,255,255,255,0,0,1,136,129,32,0,0,63,101,159,224,0,0,0,0,0,0,200,145,32,0,1,224,37,16,0,0,8,0,0,0,0,104,145,35,0,63,0,37,3,192,0,0,0,0,0,0,40,145,49,255,192,0,37,126,255,128,0,0,0,0,0,56,145,17,0,1,254,37,64,64,127,255,224,0,0,0,28,145,17,28,255,2,37,64,0,0,1,252,0,0,0,28,147,16,151,0,2,37,64,0,0,62,3,192,0,15,76,146,16,144,0,2,37,66,255,255,224,0,126,1,249,76,146,16,144,1,194,41,71,128,0,0,0,3,255,1,76,146,16,160,6,50,41,68,0,248,0,0,0,0,1,76,146,16,161,248,18,41,68,255,143,255,240,0,0,1,84,146,16,161,0,26,41,71,128,0,0,0,0,0,1,84,146,8,161,0,25,41,64,0,0,0,0,0,0,1,20,147,8,161,0,97,42,64,0,24,0,0,0,0,1,20,145,8,161,0,193,106,64,15,216,0,7,224,0,1,20,161,8,160,129,129,75,127,248,127,0,24,63,0,7,20,161,8,160,131,1,77,96,0,1,255,240,0,255,252,36,161,8,160,198,3,69,223,248,128,0,0,0,0,0,36,163,8,160,120,126,68,112,15,128,0,0,0,0,0,36,162,24,144,31,192,84,0,0,255,192,0,0,0,0,36,178,16,159,224,0,36,0,0,0,63,254,0,0,0,36,146,16,128,0,0,52,0,7,255,224,112,0,0,0,36,146,48,128,3,255,39,255,248,0,63,255,255,255,255,228,146,33,255,252,0,96,0,0,0,192,0,0,0,0,4,146,96,0,0,0,64,7,255,255,188,0,0,0,0,4,147,64,0,0,15,127,252,0,0,3,255,252,7,255,228,255,127,255,255,248,0,0,0,0,0,0,3,248,0,28,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};
  uint8_t num_frames = sizeof(images)/15/40;
  uint8_t disp = 0x1;
  uint32_t cnt = 0;
  uint8_t adress99[167] = {disp, 0x0, 0,  0,  0,  0x6, 0xA0,
  };
  uint8_t address1[] = {0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01};
  uint8_t address111[] = {0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x09};
  uint8_t address2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0E, 0x01, 0xC8};
  uint8_t address4[] = {disp, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00};
  uint8_t address3[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0x00};
  uint8_t address9[] = {disp, 0x00, 0x00, 0x00, 0x00, 0x30, 0x02, 0x00, 0x00};
  uint8_t address5[] = {disp, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00};
  uint8_t address6[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0A, 0x01, 0x03};
  uint8_t address7[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x16, 0x01, 0x01};
  uint8_t address8[] = {disp, 0x00, 0x00, 0x00, 0x00, 0x30, 0x02, 0x00, 0x01};
  uint8_t addresses[]  = {0x6, 0x9, 0xC, 0xE, 0x10, 0x12, 0x14, 0x16, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22};
  uint32_t last_frame = 0;
  while (1)
  {
    int rx_ok = 1;
    if(!send_deviate_expect(&huart1, address1, sizeof(address1), address111, sizeof(address111), 20000)) {
      HAL_Delay(100);
      continue;
    }
    delay100();

    while(1){
      if(HAL_GetTick()-last_frame > 150){
        cnt = (cnt+1)%num_frames;
        last_frame = HAL_GetTick();
      }

      address6[7] = 0x22;
      if(!send_thing(&huart1, address6, sizeof(address6), 1000)){
        rx_ok = 0;
        break;
      }
      if(!send_thing(&huart1, address3, sizeof(address3), 1000)){
        rx_ok = 0;
        break;
      }
      uint8_t* framestart = images+cnt*40*15;
      for(int panvert=0; panvert<2; panvert++){
        for(int line=0;line<10;line++) {
          for (int col = 0; col < 7; col++) {
            adress99[7+line*16+1+8+6-col] = reverse(framestart[(line+panvert*20)*15+col]);
          }
          adress99[7+line*16+1+7] = reverse((framestart[(line+panvert*20)*15+7]&0b11110000) | framestart[(line+10+panvert*20)*15]>>4);
        }

        for(int line=0;line<10;line++) {
          for (int col = 0; col < 7; col++) {
            adress99[7+line*16+1+6-col] = reverse(framestart[(line+10+panvert*20)*15+col]<<4 | framestart[(line+10+panvert*20)*15+col+1]>>4);
          }
        }
        adress99[0] = (panvert*2)+1;
        HAL_UART_Transmit_IT(&huart1, adress99, sizeof(adress99));
        HAL_Delay(1);
      }
      memset(adress99+7, 0, 160);

      for(int panvert=0; panvert<2; panvert++){
        for(int line=0;line<10;line++) {
          for (int col = 0; col < 7; col++) {
            adress99[7+line*16+1+8+6-col] = reverse(framestart[(line+panvert*20)*15+col+7]<<4 | framestart[(line+panvert*20)*15+col+8]>>4);
          }
          adress99[7+line*16+1+7] = reverse((framestart[(line+panvert*20)*15+14]<<4) | (framestart[(line+10+panvert*20)*15+7]&0b1111));
        }

        for(int line=0;line<10;line++) {
          for (int col = 0; col < 7; col++) {
            adress99[7+line*16+1+6-col] = reverse(framestart[(line+10+panvert*20)*15+col+8]);
          }
        }
        adress99[0] = (panvert*2)+2;
        HAL_UART_Transmit_IT(&huart1, adress99, sizeof(adress99));
        HAL_Delay(1);
      }

      HAL_Delay(40);
    }

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
