//GP2 IR Sensor Header File
#include "stm32f0xx.h"
#include "main.h"

void ir_1_init(void);
int16_t* ir_1_getDist(void);
void ir_getDist2(void);
int16_t ir_2_getDist(void);
void ir_getDist(void);
void ADC_Enable(void);
int16_t get_1(void);
int16_t get_2(void);
