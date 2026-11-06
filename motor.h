#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void Motor_Init(void);
void Motor_SetLeft(int16_t speed);   /* -1000..1000, negative = reverse */
void Motor_SetRight(int16_t speed);
void Motor_Enable(void);             /* STBY = high */
void Motor_Disable(void);            /* STBY = low  */

#endif
