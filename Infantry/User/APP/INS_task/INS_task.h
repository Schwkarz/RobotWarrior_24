/**
  ****************************RM Warrior 2023****************************
  * @file       INSTask.c/h
  * @brief      姿态解算，得出欧拉角
  *             可以通过bmi088的data ready 中断完成外部触发，唤醒任务
  * @note       SPI 在陀螺仪初始化的时候需要低于2MHz，之后读取数据需低于20MHz
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/1/5        pxx              1. 未完成
  *
  @verbatim
  ==============================================================================
    (1)bmi088_spi_dma_tx未明确定义
    (2)bmi088 spi_dma传输数据接口留出，但没有编写dma传输相关函数
  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#ifndef INS_TASK_H
#define INS_TASK_H
#include "main.h"

/*-------------------温度控制PID------------------*/
#define TEMPERATURE_PID_KP 1600.0f
#define TEMPERATURE_PID_KI 0.2f
#define TEMPERATURE_PID_KD 0.0f

#define TEMPERATURE_PID_MAX_OUT   4500.0f
#define TEMPERATURE_PID_MAX_IOUT 4400.0f

#define BMI088_TEMP_PWM_MAX 5000 //控制温度设置的tim重载值

#define BMI088_TEMP_SET 26
/*--------------------------------------------------*/

#define GYRO_OFFSET_KP 0.0003f //调整这个可以调整陀螺仪校准速度，越大陀螺仪校准变化越快，但波动会变大

#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间

//是否使用BMI088温度控制，不使用注释定义
#define BMI088_USE_TEMP_CONTROL

//是否使用IST8310磁力计，不使用注释定义
// #define USE_IST8310 

//是否使用BMI088的外部中断唤醒任务，不使用注释定义
// #define BMI088_USE_DATA_READY_EXIT 

//是否使用SPI的DMA传输，不使用注释定义
// #define BMI088_USE_SPI_DMA 


#define INS_DELTA_TICK 1 //任务调用的间隔

#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS        2

//获取姿态角指针地址后，对应姿态角的地址偏移量 fp32类型
#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

extern void INSTask(void *pvParameters);
extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);
extern const fp32 *get_INS_quat_point(void);
extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_gyro_data_point(void);
extern const fp32 *get_accel_data_point(void);
extern const fp32 *get_mag_data_point(void);
extern const fp32 *get_accel_filter_point(void);

#endif
