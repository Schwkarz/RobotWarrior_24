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
#include "voltage_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "adc.h"
#include "buzzer.h"
#include "led.h"
#include "user_lib.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"

#define FULL_BATTER_VOLTAGE     25.2f
#define LOW_BATTER_VOLTAGE      22.2f   //about 20% 

#define VOLTAGE_BUZZER_CYCLE_TIME 2  //蜂鸣器断续发声周期时间 *100ms
#define VOLTAGE_BUZZER_PAUSE_TIME 1 //蜂鸣器断续发声停声时间
#define voltage_start_buzzer() buzzer_on(70, 20) //蜂鸣器报警 电源电压不足
#define voltage_buzzer_off() buzzer_off()

static fp32 calc_battery_percentage(float voltage);
void voltage_low_warning(void);


fp32 battery_voltage;//电源电压
fp32 electricity_percentage;//电池电量百分比

/**
  * @brief          电源采样和计算电源百分比
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void battery_voltage_task(void* pvParameters)
{
    vTaskDelay(1000);
    //use inner 1.2v to calbrate
    init_vrefint_reciprocal();
    while(1)
    {
        battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
        electricity_percentage = calc_battery_percentage(battery_voltage);
        // voltage_low_warning();
        vTaskDelay(100);
    }
}

static fp32 calc_battery_percentage(float voltage)
{
    fp32 percentage;
    fp32 voltage_2 = voltage * voltage;
    fp32 voltage_3 = voltage_2 * voltage;
    
    if(voltage < 19.5f)
    {
        percentage = 0.0f;
    }
    else if(voltage < 21.9f)
    {
        percentage = 0.005664f * voltage_3 - 0.3386f * voltage_2 + 6.765f * voltage - 45.17f;
    }
    else if(voltage < 25.5f)
    {
        percentage = 0.02269f * voltage_3 - 1.654f * voltage_2 + 40.34f * voltage - 328.4f;
    }
    else
    {
        percentage = 1.0f;
    }

    if(percentage < 0.0f)
    {
        percentage = 0.0f;
    }
    else if(percentage > 1.0f)
    {
        percentage = 1.0f;
    }

    //另一套公式
//    if(voltage < 19.5f)
//    {
//        percentage = 0.0f;
//    }
//    else if(voltage < 22.5f)
//    {
////        percentage = 0.05776f * (voltage - 22.5f) * (voltage_2 - 39.0f * voltage + 383.4f) + 0.5f;
//        percentage = 0.05021f * voltage_3 - 3.075f * voltage_2 + 62.77f * voltage - 427.02953125f;
//    }
//    else if(voltage < 25.5f)
//    {
////        percentage = 0.01822f * (voltage - 22.5f) * (voltage_2 - 52.05f * voltage + 637.0f) + 0.5f;
//        percentage = 0.0178f * voltage_3 - 1.292f * voltage_2 + 31.41f * voltage - 254.903125f;
//    }
//    else
//    {
//        percentage = 1.0f;
//    }

    return percentage;
}

/**
  * @brief          获取电量
  * @param[in]      void
  * @retval         电量, 单位 1, 1 = 1%
  */
uint16_t get_battery_percentage(void)
{
    return (uint16_t)(electricity_percentage * 100.0f);
}

//电量过低，蜂鸣器报警，led亮红灯
void voltage_low_warning(void)
{
    static uint8_t buzzer_time = 0;
    static uint8_t flag_led = 1;
    static uint8_t rc_ctrl_no_move = 1;


    //电源电量低，led红灯
    if(electricity_percentage < 0.2f && flag_led == 1)
    {
        led_green_off();
        led_red_on();
        flag_led = 0;
    }
    else if(electricity_percentage > 0.2f && flag_led == 0)
    {
        led_red_off();
        led_green_on();
        flag_led = 1;
    }

    //步兵无力时，蜂鸣器可以报警提醒电量过低
    rc_ctrl_no_move = gimbal_cmd_to_voltage_warning_stop();

    if(electricity_percentage < 0.2f && rc_ctrl_no_move == 1)
    {
        voltage_start_buzzer();
    }

    if(rc_ctrl_no_move == 1)
    {
        buzzer_time++;
    }

    //蜂鸣器断续发声
    if (buzzer_time > VOLTAGE_BUZZER_CYCLE_TIME && rc_ctrl_no_move == 1)
    {
        buzzer_time = 0;
    }
    if (buzzer_time > VOLTAGE_BUZZER_PAUSE_TIME && rc_ctrl_no_move == 1)
    {
        voltage_buzzer_off();
    }
}

