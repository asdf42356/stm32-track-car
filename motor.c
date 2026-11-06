#include "motor.h"

/* TB6612 pin mapping
 * AIN1=PA4  AIN2=PA5  PWMA=PA6(TIM3_CH1)
 * BIN1=PA2  BIN2=PB1  PWMB=PA7(TIM3_CH2)
 * STBY=PA3
 */

#define PWM_PERIOD  999   /* 1MHz / 1000 = 1kHz */

void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Direction pins: PA2(BIN1), PA4(AIN1), PA5(AIN2) */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* BIN2: PB1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* STBY: PA3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PWM pins: PA6(TIM3_CH1), PA7(TIM3_CH2) */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TIM3 base: 72MHz / 72 = 1MHz, period 1000 -> 1kHz PWM */
    TIM_TimeBaseStructure.TIM_Prescaler     = 71;
    TIM_TimeBaseStructure.TIM_Period        = PWM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  /* PA6 */
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  /* PA7 */

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    /* Start disabled (STBY low) */
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
}

void Motor_SetLeft(int16_t speed)
{
    if (speed > 0) {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
        GPIO_SetBits(GPIOA, GPIO_Pin_5);
        TIM_SetCompare1(TIM3, (uint16_t)speed);
    } else if (speed < 0) {
        GPIO_SetBits(GPIOA, GPIO_Pin_4);
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);
        TIM_SetCompare1(TIM3, (uint16_t)(-speed));
    } else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);
        TIM_SetCompare1(TIM3, 0);
    }
}

void Motor_SetRight(int16_t speed)
{
    if (speed > 0) {
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
        GPIO_SetBits(GPIOB, GPIO_Pin_1);
        TIM_SetCompare2(TIM3, (uint16_t)speed);
    } else if (speed < 0) {
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
        TIM_SetCompare2(TIM3, (uint16_t)(-speed));
    } else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
        TIM_SetCompare2(TIM3, 0);
    }
}

void Motor_Enable(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
}

void Motor_Disable(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
}
