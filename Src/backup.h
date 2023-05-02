#include "stm32f0xx.h"
#include "main.h"
#include "distance_front.h"
#include "BT_usart.h"
#include "color_sensor_set.h"
#include "motor.h"


void mode_Operation_back_up_plan(void);
void interupt_init(void);
void EXTI0_1_IRQHandler(void);
uint16_t backup_get_1(void);
uint16_t backup_get_2(void);