/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
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

int LG3D20_addr = 0x69;
int LG3D20_WAI = 0x0F;
int LG3D20_X = 0xA8;
int LG3D20_Y = 0xAA;
	
uint32_t I2C_read_reg(uint8_t address){
	uint32_t data = 0;

	//clearing NBYTES and SADD fields
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	//setting SADD and NBYTES fields, and setting start bit
	I2C2 -> CR2 |= (1 << 16) | (LG3D20_addr << 1);
	//WRN bit (write)
	I2C2 -> CR2 &= ~(1<<10);
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	
	while(!(I2C2 -> ISR & I2C_ISR_TXIS)); 
	I2C2 -> TXDR = address;
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2 -> CR2 |= (1 << 16) | (LG3D20_addr << 1);
	//WRN bit (read);
	I2C2 -> CR2 |= (1<<10);
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	while(!(I2C2 -> ISR & I2C_ISR_RXNE));
	data = I2C2 -> RXDR;
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	//stop bit
	//return data;
	//data = I2C2 -> RXDR;
	I2C2 -> CR2 |= (1<<14);
	
 return data;
}



void I2C_L3GD20_init(int address, int value){
	//1. Set the slave address in the SADD[7:1] bit field.
	//clearing NBYTES and SADD fields
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	//setting SADD and NBYTES fields, and setting start bit
	I2C2 -> CR2 |= (2 << 16) | (LG3D20_addr << 1);
	
	//WRN bit (write)
	I2C2 -> CR2 &= ~(1<<10);
	
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	
	while(!(I2C2 -> ISR & I2C_ISR_TXIS)); 
	I2C2 -> TXDR = address;
	
	while(!(I2C2 -> ISR & I2C_ISR_TXIS));
	I2C2 -> TXDR = value;
	
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	//stop bit
	I2C2 -> CR2 |= (1<<14);

}

void read_gyro(){
	
	//enabling x and y axes and bringing device out of power down mode
	I2C_L3GD20_init(0x20, 0x0B);
	
	int16_t gyro[2] = {0,0};
	int8_t x_raw[2] = {0,0};
	int8_t y_raw[2] = {0,0};
	
	//clearing NBYTES and SADD fields
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	//setting SADD and NBYTES fields, and setting start bit
	I2C2 -> CR2 |= (1 << 16) | (LG3D20_addr << 1);
	//WRN bit (write)
	I2C2 -> CR2 &= ~(1<<10);
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	
	//Wait until TXIS bit is set
	while(!(I2C2 -> ISR & I2C_ISR_TXIS_Pos)); 
	I2C2 -> TXDR = LG3D20_X;
	
	//Wait until transfer is complete
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	
	//Reconfigure CR2 Register
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	//reading 2 bytes
	I2C2 -> CR2 |= (2 << 16) | (LG3D20_addr << 1);
	//WRN bit (read);
	I2C2 -> CR2 |= (1<<10);
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	
	//Wait for first byte of data to be ready to read
	while(!(I2C2 -> ISR & I2C_ISR_RXNE));
	x_raw[0] = I2C2 -> RXDR;
	//wait for second byte of data to be ready to read
	while(!(I2C2 -> ISR & I2C_ISR_RXNE));
	x_raw[1] = I2C2 -> RXDR;
	
	//Wait for transfer to be complete
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	
	//stop bit
	I2C2 -> CR2 |= (1<<14);
	
	gyro[0] = (x_raw[1] << 8) | (x_raw[0] <<0);
	
	if(gyro[0] == 0) {
		GPIOC -> ODR |= 1<<8 | 1<<9;
	}
	else if(x_raw[0] > 20) {
		GPIOC -> ODR &= ~((1<<9));
		GPIOC -> ODR |= 1<<8;
	}
	else if (x_raw[0] < -20) {
		GPIOC -> ODR &= ~((1<<8));
		GPIOC -> ODR |= 1<<9;
	}
		
	//clearing NBYTES and SADD fields
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	//setting SADD and NBYTES fields, and setting start bit
	I2C2 -> CR2 |= (1 << 16) | (LG3D20_addr << 1);
	//WRN bit (write)
	I2C2 -> CR2 &= ~(1<<10);
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	
	//Wait until TXIS bit is set
	while(!(I2C2 -> ISR & I2C_ISR_TXIS_Pos)); 
	I2C2 -> TXDR = LG3D20_Y;
	
	//Wait until transfer is complete
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	
	//Reconfigure CR2 Register
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	//reading 2 bytes
	I2C2 -> CR2 |= (2 << 16) | (LG3D20_addr << 1);
	//WRN bit (read);
	I2C2 -> CR2 |= (1<<10);
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	
	//Wait for first byte of data to be ready to read
	while(!(I2C2 -> ISR & I2C_ISR_RXNE));
	y_raw[0] = I2C2 -> RXDR;
	//wait for second byte of data to be ready to read
	while(!(I2C2 -> ISR & I2C_ISR_RXNE));
	y_raw[1] = I2C2 -> RXDR;
	
	//Wait for transfer to be complete
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	
	//stop bit
	I2C2 -> CR2 |= (1<<14);
	
	gyro[1] = (y_raw[1] << 8) | (y_raw[0] << 0);
	
	if(gyro[1] == 0) {
		GPIOC -> ODR |= (1<<6) | (1<<7);
	}
	else if(gyro[1] > 20) {
		GPIOC -> ODR &= ~((1<<6));
		GPIOC -> ODR |= 1<<7;
	}
	else if (gyro[1] < -20) {
		GPIOC -> ODR &= ~((1<<7));
		GPIOC -> ODR |= 1<<6;

	}
}

int main(void)
{
	
	SystemClock_Config();
	
	//Enabling GPIOB, GPIOC, and I2C2 peripherals in RCC
	RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC -> APB1ENR |= RCC_APB1ENR_I2C2EN;
  
	//Setting PB13 and PB11 to alternate function mode and PB14 to output mode
	GPIOB -> MODER &= ~((1<<29) | (1<<28) | (1<<27) | (1<<26) | (1<<23) | (1<<22));
	GPIOB -> MODER |= (1<<28) | (1<<27) | (1<<23) ;
	
	//Setting PC0 to output mode (also LEDs for debugging)
	GPIOC -> MODER &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16) | (1<<15) | (1<<14) | (1<<13) | (1<<12) | (1<<1) | (1<<0));
	GPIOC -> MODER |= (1<<18) | (1<<16) | (1<<14) | (1<<12) | (1<<0);
	
	GPIOC -> PUPDR &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16) | (1<<15) | (1<<14) | (1<<13) | (1<<12));
	GPIOC -> OSPEEDR &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16) |(1<<15) | (1<<14) | (1<<13) | (1<<12));
	
	//Setting PB13 and PB11 to open drain mode and PB14 to push pull mode
	GPIOB -> OTYPER &= ~((1<<14) | (1<<13) | (1<<11));
	GPIOB -> OTYPER |= (1<<13) | (1<<11);
	
	//Configuring PC0 to push pull mode
	GPIOC -> OTYPER &= ~((1<<0));
	
	//Setting PC0 and PB14 to high
	GPIOB -> ODR |= (1<<14);
	GPIOC -> ODR |= (1<<0);
	
	//Setting PB13 to AF5 and PB11 to AF1
	GPIOB -> AFR[1] |=   (0x05 << GPIO_AFRH_AFSEL13_Pos) | (0x01 <<GPIO_AFRH_AFSEL11_Pos);
	
	//setting TIMINGR to use 100kHz and standard mode I2C
	I2C2 -> TIMINGR |= (0x01<<28) | (0x4<<20) | (0x2<<16) | (0xF<<8) | (0x13<<0); 
	
	//enabling I2C2 peripheral
	I2C2 -> CR1 |= 1<<0;
	
	//enabling x and y axes and bringing device out of power down mode
	I2C_L3GD20_init(0x20, 0x0B);
	
	int16_t gyro[2] = {0,0};
	int8_t x_raw[2] = {0,0};
	int8_t y_raw[2] = {0,0};
	
	while (1)
  {

	
	//clearing NBYTES and SADD fields
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	//setting SADD and NBYTES fields, and setting start bit
	I2C2 -> CR2 |= (1 << 16) | (LG3D20_addr << 1);
	//WRN bit (write)
	I2C2 -> CR2 &= ~(1<<10);
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	
	//Wait until TXIS bit is set
	while(!(I2C2 -> ISR & I2C_ISR_TXIS_Pos)); 
	I2C2 -> TXDR = LG3D20_X;
	
	//Wait until transfer is complete
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	
	//Reconfigure CR2 Register
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	//reading 2 bytes
	I2C2 -> CR2 |= (2 << 16) | (LG3D20_addr << 1);
	//WRN bit (read);
	I2C2 -> CR2 |= (1<<10);
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	
	//Wait for first byte of data to be ready to read
	while(!(I2C2 -> ISR & I2C_ISR_RXNE));
	x_raw[0] = I2C2 -> RXDR;
	//wait for second byte of data to be ready to read
	while(!(I2C2 -> ISR & I2C_ISR_RXNE));
	x_raw[1] = I2C2 -> RXDR;
	
	//Wait for transfer to be complete
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	
	//stop bit
	I2C2 -> CR2 |= (1<<14);
	
	gyro[0] = (((int16_t) x_raw[0] << 8) | (x_raw[1] <<0));
	
	if(gyro[0] == 0) {
		GPIOC -> ODR |= 1<<8 | 1<<9;
	}
	else if(x_raw[0] > 38) {
		GPIOC -> ODR &= ~((1<<9));
		GPIOC -> ODR |= 1<<8;
	}
	else if (x_raw[0] < -38) {
		GPIOC -> ODR &= ~((1<<8));
		GPIOC -> ODR |= 1<<9;
	}
	//HAL_Delay(100);
	//clearing NBYTES and SADD fields
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	//setting SADD and NBYTES fields, and setting start bit
	I2C2 -> CR2 |= (1 << 16) | (LG3D20_addr << 1);
	//WRN bit (write)
	I2C2 -> CR2 &= ~(1<<10);
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	
	//Wait until TXIS bit is set
	while(!(I2C2 -> ISR & I2C_ISR_TXIS_Pos)); 
	I2C2 -> TXDR = LG3D20_Y;
	
	//Wait until transfer is complete
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	
	//Reconfigure CR2 Register
	I2C2 -> CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	//reading 2 bytes
	I2C2 -> CR2 |= (2 << 16) | (LG3D20_addr << 1);
	//WRN bit (read);
	I2C2 -> CR2 |= (1<<10);
	//Start bit
	I2C2 -> CR2 |= (1<<13);
	
	//Wait for first byte of data to be ready to read
	while(!(I2C2 -> ISR & I2C_ISR_RXNE));
	y_raw[0] = I2C2 -> RXDR;
	//wait for second byte of data to be ready to read
	while(!(I2C2 -> ISR & I2C_ISR_RXNE));
	y_raw[1] = I2C2 -> RXDR;
	
	//Wait for transfer to be complete
	while(!(I2C2 -> ISR & I2C_ISR_TC));
	
	//stop bit
	I2C2 -> CR2 |= (1<<14);
	
	gyro[1] = (((int16_t)(y_raw[0] << 8)) | (y_raw[1] << 0));
	
	if(gyro[1] == 0) {
		GPIOC -> ODR |= (1<<6) | (1<<7);
	}
	else if(gyro[1] > 38) {
		GPIOC -> ODR &= ~((1<<6));
		GPIOC -> ODR |= 1<<7;
	}
	else if (gyro[1] < -38) {
		GPIOC -> ODR &= ~((1<<7));
		GPIOC -> ODR |= 1<<6;

	}
		HAL_Delay(100);
		//call read gyro function
	
		
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
