#include "main.h"


// Best success with ATIME register set to 0x00 for 614ms time (ATIME Register)
// 0x80 set to 0x03 Enabling the AEN and PON (Enable Register)
// 


//static uint8_t enable_register = 0x00; // enable register
//static uint8_t integration_register = 0x01; // timing register


// 0x44 // address for TCS34725 (0x12 is ID register - read only)

// 0x00 -> 0x03 // 0011 set enable register bit 1 to 1 (AEN) to enable and PON (Bit 0 to 1 enable) will begin RGBC cycle

// Setting of integration time
//0x01 -> 0xD6; // 100ms integration time ATIME register (0x01)

// Data for blue registers (read only)
// 0x1A // 7:0 // Blue data low byte BDATA
// 0x1B // 7:0 // Blue data high byte BDATAH

void SystemClock_Config(void);

// Function prototypes and static vars
void Tx_1_Byte(uint8_t transmission);
void Tx_2_Bytes(uint8_t transmission_1, uint8_t transmission_2);

uint8_t Rx_1_Byte(void);
uint8_t* Rx_2_Bytes(void);


static uint8_t slave_address = 0x29; // slave address of color sensor
static uint8_t enable_address = 0x80;// sensor address for enable (0x00) + the 1 in front for the command register (1000 0000) ORed with 0x80
static uint8_t timing_address = 0x81;// sensor address for timing
static uint8_t enable_value = 0x03;// what we want enable register to be to enable PON and AEN
static uint8_t timing_value = 0x00;// what we want enable register to be
static uint8_t blue_data_low_address = 0x9A; // X data registers (read both registers in same transaction) ORed with 0x80
static uint8_t blue_data_high_address = 0x9B; // Y data registers (read both registers in same transaction) ORed with 0x80



int main(void)
{
  HAL_Init();
  SystemClock_Config();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  
  // Initialize GPIO pins - same as part 1
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enables the GPIOC peripheral clock
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // GPIOB
  GPIOB->MODER |= 0x18800000; // 11, 13 alternate function, 14 Output
  GPIOB->OTYPER |= 0x2800; // Open-drain on 11 and 13 
  GPIOB->AFR[1] |= (0x01 << GPIO_AFRH_AFSEL11_Pos) | (0x05 << GPIO_AFRH_AFSEL13_Pos); // PB11 AF1 - set as SDA , PB13 AF5 - set as SCL
  GPIOC->MODER |= 0x55001; // pin 0 output, LED on pin 6,7,8,9 output
  
  // Initialize I2C
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; 
  I2C2->TIMINGR = 0x10420F13; // 100 kHz, 5 usL, 10us H, 500 ns SDADEL, 1250 ns SCLDEL - as defined in the manual
  I2C2->CR1 |= I2C_CR1_PE; // locking PE bit 0
  
  // Initialize Pins
	GPIOB->BSRR = GPIO_PIN_14;
	GPIOC->BSRR = GPIO_PIN_0;	
  
  while (1)
  {  
    volatile uint8_t blue_low_read;
    volatile uint8_t blue_high_read;
    volatile uint16_t blue;
    
    
    Tx_2_Bytes(enable_address, enable_value); // Set enable register to 0x0B
    I2C2->CR2 |= I2C_CR2_STOP; 
    //Tx_2_Bytes(timing_address, timing_value); // Set enable register to 0x0B
    //I2C2->CR2 |= I2C_CR2_STOP; 
    while (1)
    {     
      Tx_1_Byte(blue_data_low_address); // telling which register to read from
      blue_low_read = Rx_1_Byte();
      I2C2->CR2 |= I2C_CR2_STOP;

      Tx_1_Byte(blue_data_high_address); // telling which register to read from
      blue_high_read = Rx_1_Byte();
      I2C2->CR2 |= I2C_CR2_STOP;

      blue = (blue_high_read << 8) | blue_low_read; // high then low?
      
      if (blue > 6000)
      {
				GPIOC->BRR = GPIO_PIN_8;
				GPIOC->BSRR = GPIO_PIN_9; // Light green
      }
      if (blue < 6000)
      {
				GPIOC->BRR = GPIO_PIN_9;
				GPIOC->BSRR = GPIO_PIN_8; // Light orange
      }

      HAL_Delay(10); // read registers every 100 ms (default)
    }
  }
}



void Tx_1_Byte(uint8_t transmission)
{
  I2C2->CR2 = (slave_address << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
  while (((I2C2->ISR & I2C_ISR_TXIS_Msk) == 0) && ((I2C2->ISR & I2C_ISR_NACKF_Msk) == 0)) 
  { } // wait
  if (I2C2->ISR & I2C_ISR_TXE_Msk) // TX ready
  {
    I2C2->TXDR = transmission;
    while ((I2C2->ISR & I2C_ISR_TC_Msk) == 0) // waiting for TC flag to be set
    { } // wait
    return;
  }
  else
  {
    GPIOC->BSRR = GPIO_PIN_9; // Light green LED if error
  }
}

void Tx_2_Bytes(uint8_t transmission_1, uint8_t transmission_2)
{
  I2C2->CR2 = (slave_address << 1) | (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
  while (((I2C2->ISR & I2C_ISR_TXIS_Msk) == 0) && ((I2C2->ISR & I2C_ISR_NACKF_Msk) == 0)) 
  { } // wait
  if (I2C2->ISR & I2C_ISR_TXE_Msk) // TX ready
  {
    I2C2->TXDR = transmission_1;
  }
  else
  {
    GPIOC->BSRR = GPIO_PIN_9; // Light green LED if error
  }
  while (((I2C2->ISR & I2C_ISR_TXIS_Msk) == 0) && ((I2C2->ISR & I2C_ISR_NACKF_Msk) == 0)) 
  { } // wait
  if (I2C2->ISR & I2C_ISR_TXE_Msk) // TX ready
  {
    I2C2->TXDR = transmission_2;
    while ((I2C2->ISR & I2C_ISR_TC_Msk) == 0) // waiting for TC flag to be set
    { } // wait
    return;
  }
  else
  {
    GPIOC->BSRR = GPIO_PIN_9; // Light green LED if error
  }
}

uint8_t Rx_1_Byte(void) 
{
  uint8_t data;
  
  I2C2->CR2 = (slave_address << 1) | (1 << I2C_CR2_NBYTES_Pos) | (1 << I2C_CR2_RD_WRN_Pos) | I2C_CR2_START;
  while (((I2C2->ISR & I2C_ISR_RXNE_Msk) == 0) && ((I2C2->ISR & I2C_ISR_NACKF_Msk) == 0)) 
  { } // wait
  if (I2C2->ISR & I2C_ISR_RXNE_Msk)
  {
    data = I2C2->RXDR;
    while ((I2C2->ISR & I2C_ISR_TC_Msk) == 0)
    { } // wait
  }
  else
  {
    GPIOC->BSRR = GPIO_PIN_9; // Light green LED if error
  }
  return data;
}

uint8_t* Rx_2_Bytes(void) 
{
  static uint8_t data[2];
  
  I2C2->CR2 = (slave_address << 1) | (2 << I2C_CR2_NBYTES_Pos) | (1 << I2C_CR2_RD_WRN_Pos) | I2C_CR2_START;
  while (((I2C2->ISR & I2C_ISR_RXNE_Msk) == 0) && ((I2C2->ISR & I2C_ISR_NACKF_Msk) == 0)) 
  { } // wait
  if (I2C2->ISR & I2C_ISR_RXNE_Msk)
  {
    data[0] = I2C2->RXDR;
  }
  else
  {
    GPIOC->BSRR = GPIO_PIN_9; // Light green LED if error
  }
  while (((I2C2->ISR & I2C_ISR_RXNE_Msk) == 0) && ((I2C2->ISR & I2C_ISR_NACKF_Msk) == 0)) 
  { } // wait
  if (I2C2->ISR & I2C_ISR_RXNE_Msk)
  {
    data[1] = I2C2->RXDR;
    while ((I2C2->ISR & I2C_ISR_TC_Msk) == 0)
    { } // wait
  }
  else
  {
    GPIOC->BSRR = GPIO_PIN_9; // Light green LED if error
  }
  return data;
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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


void Error_Handler(void)
{

}





