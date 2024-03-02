/**
  *******************************RM Warrior 2023********************************
  * @file       voltage_task.c/h
  * @brief      24v power voltage ADC task, get voltage and calculate electricity
  *             percentage.24电源电压ADC任务,获取电压并且计算电量百分比.
  * @note       when power is not derectly link to delelopment, please change VOLTAGE_DROP
  *             当电源不直连开发板,请修改VOLTAGE_DROP
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/1/5        pxx              ......
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *******************************RM Warrior 2023********************************
  */
#ifndef VOLTAGE_TASK_H
#define VOLTAGE_TASK_H
#include "main.h"

#define VOLTAGE_DROP            0.00f

extern void battery_voltage_task(void* pvParameters);
extern uint16_t get_battery_percentage(void);

#endif
