/**
  ****************************RM Warrior 2023****************************
  * @file       freeRTOS_middle.c/h
  * @brief      freeRTOS的中间层，将滴答计时中断和统计任务用时的接口函数放到这里.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/2/         pxx              ......
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#include "FreeRTOS_Middleware.h"
#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "timer.h"

// //设置调度中断定时器配置
// void vPortSetupTimerInterrupt(void)
// {
// }

extern void xPortSysTickHandler(void);
void SysTick_Handler(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    xPortSysTickHandler();
  }
}

// 利用tim3作为任务统计用时，所有任务目前测试cpu利用率
volatile uint64_t FreeRTOSRunTimeTicks = 0;

void ConfigureTimeForRunTimeStats(void)
{
  TIM3_Init();
  FreeRTOSRunTimeTicks = 0;
}

void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    FreeRTOSRunTimeTicks++;
    TIM_ClearFlag(TIM3, TIM_IT_Update);
  }
}
