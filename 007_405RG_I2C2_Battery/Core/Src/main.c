/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */



/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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

	MX_DMA_Init();

	MX_I2C2_Init();

	MX_SPI1_Init();
	MX_SPI2_Init();

	MX_TIM1_Init();
	MX_TIM4_Init();

	MX_UART5_Init();
	MX_USART2_UART_Init();
	MX_USART6_UART_Init();

	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	uint8_t *_data_tx_2 = (uint8_t*) "Hello world 2";
	uint8_t *_data_tx_5 = (uint8_t*) "Hello world 5";
	uint8_t *_data_tx_6 = (uint8_t*) "Hello world 6";

	typedef struct {
		uint8_t register_address;
		uint8_t size;
	} I2C_Data;

#define I2C_READS_SIZE		3
	I2C_Data _i2c_reads[] = {
			{0x01, 2},
			{0x03, 4},
			{0x12, 2}
	};

	while (1) {
#define I2C_BATTERY	0x0B
#define DATA_BUFFER_SIZE 256

		uint8_t _device_address = I2C_BATTERY << 1;
		uint8_t _register_address = 0x01;
		uint8_t _data_buffer[DATA_BUFFER_SIZE];
		uint8_t _receive_size = 0;


// read ALL registers from 0x00 to 0xFE
		_receive_size = 8;
		for (uint8_t _register_index = 0; _register_index < 255; _register_index++) {
			memset(_data_buffer, 0, DATA_BUFFER_SIZE);
			HAL_I2C_Master_Transmit(&hi2c2, _device_address, &_register_index, 1, 100);
			HAL_I2C_Master_Receive(&hi2c2, _device_address, _data_buffer, _receive_size, 100);

			HAL_Delay(10);
		} // for (uint8_t _register_index = 0; _register_index < 255; _register_index++) {


// read only the three known registers
//		while(1) {
//			for (uint8_t _i2c_data_index = 0; _i2c_data_index < I2C_READS_SIZE; _i2c_data_index++) {
//				memset(_data_buffer, 0, DATA_BUFFER_SIZE);
//
//				_register_address = _i2c_reads[_i2c_data_index].register_address;
//				_receive_size = _i2c_reads[_i2c_data_index].size;
//
//				HAL_I2C_Master_Transmit(&hi2c2, _device_address, &_register_address, 1, 100);
//
//				HAL_I2C_Master_Receive(&hi2c2, _device_address, _data_buffer, _receive_size, 100);
//			} // for (uint8_t _i2c_data_index = 0; _i2c_data_index < I2C_READS_SIZE; _i2c_data_index++) {
//
//			HAL_Delay(100);
//		} // while(1) {


		while(1) {
			asm("nop");
		}


//		HAL_UART_Transmit(&huart2, _data_tx_2, 13, 1000);
//		HAL_UART_Transmit(&huart5, _data_tx_5, 13, 1000);
//		HAL_UART_Transmit(&huart6, _data_tx_6, 13, 1000);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 24;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
