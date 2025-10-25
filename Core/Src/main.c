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
#include <stdio.h>
#include <string.h>
#include "sx1278.h"

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t lora_read(uint8_t addr)
{
    uint8_t reg_val = 0;
    uint8_t tx[2] = { addr & 0x7F, 0x00 }; // send address, then dummy
    uint8_t rx[2] = {0};
    HAL_Delay(10);

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    reg_val = rx[1]; // second byte is the register value
    return reg_val;
}


void lora_write(uint8_t addr, uint8_t data){
    uint8_t address_byte = (addr & 0x7F) | 0x80; // Set bit 7 for write operation (wnr=1)
    uint8_t tx_buffer[2] = {address_byte, data};

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 0);
    HAL_Delay(10);

    // Transmit address byte (with write bit) followed by data
    HAL_SPI_Transmit(&hspi1, tx_buffer, 2, 1000);
    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 1);
    HAL_Delay(10);
}

void print(char *msg){
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 500);
	HAL_Delay(10);
}

void lora_set_lora_mode(void)
{
    uint8_t op_mode;

    // 1. Read current RegOpMode
    op_mode = lora_read(REG_OP_MODE);

    // 2. Put device into sleep mode (Mode bits = 000)
    op_mode &= 0xF8;       // Clear Mode[2:0]
    op_mode |= 0x00;       // Set to Sleep mode
    lora_write(REG_OP_MODE, op_mode);

    HAL_Delay(10);         // Small delay for mode change

    // 3. Set LongRangeMode = 1 (LoRa mode)
    op_mode = lora_read(REG_OP_MODE);
    op_mode |= (1 << 7);   // Set bit 7 = 1
    lora_write(REG_OP_MODE, op_mode);

    // 4. Return to Standby mode (Mode bits = 001)
    op_mode &= 0xF8;       // Clear Mode bits
    op_mode |= 0x01;       // Set to Standby
    lora_write(REG_OP_MODE, op_mode);

    HAL_Delay(10);
}

#define REG_OP_MODE     0x01

void lora_set_mode_tx(void)
{
    uint8_t op_mode;

    // 1. Read current operation mode register
    op_mode = lora_read(REG_OP_MODE);

    // 2. Ensure LongRangeMode is LoRa (bit 7 = 1)
    if (!(op_mode & (1 << 7))) {
        print("Not in LoRa mode! Aborting TX mode set.\r\n");
        return;
    }

    // 3. Clear old mode bits (2:0) and set to TX mode (011)
    op_mode &= 0xF8;   // clear bits 2–0
    op_mode |= 0x03;   // set bits to 011 → Transmit mode
    lora_write(REG_OP_MODE, op_mode);

    HAL_Delay(1); // small delay for the mode transition
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  uint8_t tx_data[255];
  uint8_t rx_data[255];
  char string[64];
  uint8_t val;

  lora_set_lora_mode();
  lora_write(0x00, 0xAE);
  lora_set_mode_tx();
  lora_write(0x00, 0xEA);
  val = lora_read(REG_OP_MODE);
  snprintf(string, sizeof(string), "RegOpMode: 0x%02X\r\n", val);
  print(string);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
	  // Fill the buffer with test pattern (e.g., 0x00, 0x01, 0x02, ...)
	  /*
	  for (uint16_t i = 0; i < 255; i++) {
	      tx_data[i] = i;
	  }

	  // Write 255 bytes into the FIFO
	  for (uint16_t i = 0; i < 250; i++) {
	      lora_write(0x00, tx_data[i]);
	  }

	  // Optional: print confirmation
	  print("255 bytes written to FIFO\r\n");

	  // Read 255 bytes back from FIFO
	  for (uint16_t i = 0; i < 254; i++) {
	      rx_data[i] = lora_read(0x00);
	  }

	  // Compare and print result
	  for (uint16_t i = 0; i < 255; i++) {
	      snprintf(string, sizeof(string), "Byte %03d: TX=0x%02X  RX=0x%02X\r\n", i, tx_data[i], rx_data[i]);
	      print(string);
	  }
	  */

	  /* FIFORegAdrrPointer increment
	  val = lora_read(0x0D);
      snprintf(string, sizeof(string), "Reg1: 0x%02X\r\n", val);
      print(string);

      lora_write(0x00, 0xEE);

	  val = lora_read(0x0D);
      snprintf(string, sizeof(string), "Reg2: 0x%02X\r\n", val);
      print(string);
*/



      HAL_Delay(10000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
