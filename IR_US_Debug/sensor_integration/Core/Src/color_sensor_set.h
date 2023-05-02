#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

void Tx_1_Byte(uint8_t transmission);
void Tx_2_Bytes(uint8_t transmission_1, uint8_t transmission_2);

uint8_t Rx_1_Byte(void);
uint8_t* Rx_2_Bytes(void);

void color_sensor_init(void);
uint16_t getColor(void);
