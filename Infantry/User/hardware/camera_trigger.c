#include "camera_trigger.h"
#include "stm32f4xx.h"

//PI7 TIM8_CH3
void camera_trigger_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    GPIO_PinAFConfig(GPIOI, GPIO_PinSource7, GPIO_AF_TIM8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOI, &GPIO_InitStructure);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM8, DISABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 168 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //168MHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Camera_NVIC;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse = 10000 - 1;
    
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    //TIM_Cmd(TIM8, ENABLE);
}

void Camera_trigger_set(uint16_t time_ms)
{
    TIM8->ARR = time_ms * 1000 - 1;
    TIM_SetCompare3(TIM8, (TIM8->ARR + 1) / 2 - 1);
}

void Camera_trigger_start(void)
{
    TIM_Cmd(TIM8, ENABLE);
}

void Camera_trigger_stop(void)
{
    TIM_Cmd(TIM8, DISABLE);
}
