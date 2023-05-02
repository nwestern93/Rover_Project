//GP2 IR Sensor Header File
#include "stm32f0xx.h"
#include "main.h"

void ir_1_init(void);
int16_t ir_1_getDist(void);
int16_t ir_2_getDist(void);
void ir_getDist(void);
void ADC_Enable(void);