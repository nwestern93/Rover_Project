/* Supplemental information for understanding configuration of the color sensor
 
 Each register needs to be ORed with 0x80 as the command register requires a 1 in front (i.e. 1000 0000)
 enable_register = 0x00; // enable register
 integration_register = 0x01; // timing register

 0x44 is the part number ID for TCS34725 (0x12 is ID register - read only)

 0x00 -> 0x03 // 0011 set enable register bit 1 to 1 (AEN) to enable and PON (Bit 0 to 1 enable) will begin RGBC cycle

 Setting of integration time
 0x01 -> 0xD6; // 100ms integration time ATIME register (0x01)

 Data for blue registers (read only)
 0x1A // 7:0 // Blue data low byte BDATA
 0x1B // 7:0 // Blue data high byte BDATAH
 The blue data is concatenated as a 16 bit value (2 bytes) and the threshold value is determined relative
 to a baseline color
*/

#include "color_sensor_set.h"


static uint8_t slave_address = 0x29; // slave address of color sensor
static uint8_t enable_address = 0x80;// sensor address for enable (0x00) + the 1 in front for the command register (1000 0000) ORed with 0x80
static uint8_t timing_address = 0x81;// sensor address for timing
static uint8_t enable_value = 0x03;// what we want enable register to be to enable PON and AEN (0000 0011)
static uint8_t timing_value = 0x00;// what we want enable register to be (corresponds with integration time of 614ms) [0xFF is 2.4 ms]
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
