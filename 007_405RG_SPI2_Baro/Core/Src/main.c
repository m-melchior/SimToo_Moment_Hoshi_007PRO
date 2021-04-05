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
#include "fbm.h"
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
/* Private variable */
// part of the fbm_320 driver example code
int32_t real_p, real_t, altitude;
float actual_p, actual_t;


void bit_bang() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
//    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

// toggle MISO 50 times
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // CLK
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // MISO
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0); // MOSI
//	HAL_Delay(3000);
//
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1); // CLK
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1); // MISO
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // MOSI
//	HAL_Delay(4);
//
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0); // MOSI
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // MISO
//	HAL_Delay(1);
//
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // MOSI
//	HAL_Delay(1);
//
//	for (int i=0; i<50; i++) {
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1); // MISO
//		HAL_Delay(50);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); // MISO
//		HAL_Delay(50);
//	}
//	HAL_Delay(144);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // CLK
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0); // MOSI
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS

	HAL_Delay(1000);

	uint8_t _data_tx[] = { 0x00, 0x00, 0x81 };
	uint8_t _byte = 0;
	uint8_t _bit;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS

	for (uint8_t _byte_index = 0; _byte_index < 3; _byte_index++) {
		_byte = _data_tx[_byte_index];

		for (uint8_t _bit_index = 0; _bit_index < 8; _bit_index++) {
			HAL_Delay(1);
			_bit = (_byte >> (7 - _bit_index)) & 0x01;
			if (_bit == 1) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // MOSI
			} else {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0); // MOSI
			}
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1); // CLK
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // CLK
		}
	}
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS

	HAL_Delay(3000);

	_data_tx[0] = 0x80;
	_data_tx[1] = 0x6B;
	_data_tx[2] = 0x00;

	while (1) {

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS

		for (uint8_t _byte_index = 0; _byte_index < 3; _byte_index++) {
			_byte = _data_tx[_byte_index];
			for (uint8_t _bit_index = 0; _bit_index < 8; _bit_index++) {
				HAL_Delay(1);
				_bit = (_byte >> (7 - _bit_index)) & 0x01;
				if (_bit == 1) {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1); // MOSI
				} else {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0); // MOSI
				}
				HAL_Delay(1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1); // CLK
				HAL_Delay(1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); // CLK
			}
		}
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS

		HAL_Delay(1000);
	}
} // void bit_bang() {



void do_hal() {
	uint8_t _data_tx[3];
	uint8_t _data_rx = 0;

	HAL_Delay(1000);

	_data_tx[0] = 0x00 | 0x00;
	_data_tx[1] = 0x00;
	_data_tx[2] = 0x81;

//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS
//	HAL_Delay(1);
//	for (uint8_t _byte_index = 0; _byte_index < 3; _byte_index++) {
//		HAL_SPI_Transmit(&hspi2, &_data_tx[_byte_index], 1, 1000);
//	}
	HAL_SPI_Transmit(&hspi2, _data_tx, 3, 1000);

//	HAL_Delay(1);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS

	HAL_Delay(1000);

	_data_tx[0] = 0x80 | 0x00;
	_data_tx[1] = 0x6B;

	while(1) {
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS
//		HAL_Delay(1);

//		for (uint8_t _byte_index = 0; _byte_index<2; _byte_index++) {
//			HAL_SPI_Transmit(&hspi2, &_data_tx[_byte_index], 1, 1000);
//			HAL_Delay(1);
//		}
		HAL_SPI_Transmit(&hspi2, _data_tx, 2, 1000);

		HAL_SPI_Receive(&hspi2, &_data_rx, 1, 100);

//		HAL_Delay(1);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS

		HAL_Delay(1000);
	}
} // void do_hal() {



void read_pressure() {
	HAL_StatusTypeDef _status = HAL_OK;
	uint8_t _data_tx[3];
	uint8_t _data_rx[3];

	_data_tx[0] = 0x00;
	_data_tx[1] = 0xf4;
	_data_tx[2] = 0xf4;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS
	HAL_Delay(1);
	_status = HAL_SPI_Transmit(&hspi2, _data_tx, 3, 1000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS

	HAL_Delay(7);

	_data_tx[0] = 0xc0;
	_data_tx[1] = 0xf8;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS
	HAL_Delay(1);

	_status = HAL_SPI_Transmit(&hspi2, _data_tx, 2, 1000);

	_status = HAL_SPI_Receive(&hspi2, _data_rx, 3, 1000);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS
} // void read_pressure() {



void read_temperature() {
	HAL_StatusTypeDef _status = HAL_OK;
	uint8_t _data_tx[3];
	uint8_t _data_rx[3];

	_data_tx[0] = 0x00;
	_data_tx[1] = 0xf4;
	_data_tx[2] = 0x2e;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS
	HAL_Delay(1);

	_status = HAL_SPI_Transmit(&hspi2, _data_tx, 3, 1000);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS


//	HAL_Delay(2);


	_data_tx[0] = 0xc0;
	_data_tx[1] = 0xf8;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS
	HAL_Delay(1);

	_status = HAL_SPI_Transmit(&hspi2, _data_tx, 2, 1000);

	_status = HAL_SPI_Receive(&hspi2, _data_rx, 3, 1000);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS
} // void read_temperature() {



void spi_bang() {
	HAL_StatusTypeDef _status = HAL_OK;
	uint8_t _data_tx[3];
	uint8_t _data_rx[3];

	_data_tx[0] = 0x00;
	_data_tx[1] = 0x00;
	_data_tx[2] = 0x81;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS
	HAL_Delay(1);
	_status = HAL_SPI_Transmit(&hspi2, _data_tx, 3, 1000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS

	HAL_Delay(200);


//	_data_tx[0] = 0x00;
//	_data_tx[1] = 0xa6;
//	_data_tx[2] = 0x7d; // 0xa9;
//
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS
//	HAL_Delay(1);
//	_status = HAL_SPI_Transmit(&hspi2, _data_tx, 3, 1000);
//	HAL_Delay(1);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS
//
//	HAL_Delay(200);



	_data_tx[0] = 0x80;
	_data_tx[1] = 0xa6;
	uint8_t _byte_tx = 0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0); // CSS
	HAL_Delay(1);

	_status = HAL_SPI_Transmit(&hspi2, _data_tx, 2, 1000);

	_status = HAL_SPI_Receive(&hspi2, &_byte_tx, 1, 1000);

	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); // CSS

	_byte_tx &= ~(FBM320_P_CONFIG_REG_GAIN_MAK);
	_byte_tx |= FBM320_P_CONFIG_REG_GAIN_X32;

	_status = 0;


	while(1) {
		read_pressure();

		read_temperature();
	}
} // void bit_bang() {

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



//	bit_bang();			// simple and pure, needs "MX_SPI1_Init() to be commented out above

//	spi_bang();		// more to the recorded comms
//	do_hal();			// do it all with HAL


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	fbm320_init(); //fbm320 initiation
	HAL_Delay(1);

	while (1) {

		fbm320_update_data(); //Updating fbm320 data
		if (fbm320_update_rdy) {
			actual_p = fbm320_read_pressure();//If you need the pressure value read is in uint of Pa, use this function.
			actual_t = fbm320_read_temperature();//If you need the temperature value read is in unit of degree Celsius, use this function.

			/* This function read pressure and temperature values. Pressure uint:0.125Pa, Temperature unit:0.01 degree Celsius */
			fbm320_read_data(&real_p, &real_t);
			altitude = fbm325_get_altitude(real_p);			//This function converts pressure value to altitude in unit of millimeter(mm).
			fbm320_update_rdy = 0;
		}

		HAL_Delay(1);

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
