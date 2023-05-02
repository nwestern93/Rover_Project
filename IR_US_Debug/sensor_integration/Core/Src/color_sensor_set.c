
#include "color_sensor_set.h"


static uint8_t slave_address = 0x29; // slave address of color sensor
static uint8_t enable_address = 0x80;// sensor address for enable (0x00) + the 1 in front for the command register (1000 0000) ORed with 0x80
//static uint8_t timing_address = 0x81;// sensor address for timing
static uint8_t enable_value = 0x03;// what we want enable register to be to enable PON and AEN
//static uint8_t timing_value = 0x00;// what we want enable register to be
static uint8_t blue_data_low_address = 0x9A; // X data registers (read both registers in same transaction) ORed with 0x80
static uint8_t blue_data_high_address = 0x9B; // Y data registers (read both registers in same transaction) ORed with 0x80


void color_sensor_init(void){
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
	
	
	Tx_2_Bytes(enable_address, enable_value); // Set enable register to 0x0B
  I2C2->CR2 |= I2C_CR2_STOP; 
}

uint16_t getColor(void){
	
	volatile uint8_t blue_low_read;
  volatile uint8_t blue_high_read;
  volatile uint16_t blue;
	Tx_1_Byte(blue_data_low_address); // telling which register to read from
  blue_low_read = Rx_1_Byte();
  I2C2->CR2 |= I2C_CR2_STOP;

  Tx_1_Byte(blue_data_high_address); // telling which register to read from
  blue_high_read = Rx_1_Byte();
  I2C2->CR2 |= I2C_CR2_STOP;

  blue = (blue_high_read << 8) | blue_low_read; // high then low?
	
	return blue;
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
