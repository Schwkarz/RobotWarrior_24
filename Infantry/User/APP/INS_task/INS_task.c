/**
  *******************************RM Warrior 2023********************************
  * @file       INSTask.c/h
  * @brief      姿态解算，得出欧拉角
  *             可以通过bmi088的data ready 中断完成外部触发，唤醒任务
  *             可以通过pwm控制电阻加热，抑制温漂
  * @note       SPI 在陀螺仪初始化的时候需要低于2MHz，之后读取数据需低于20MHz
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/1/5        pxx              ......
  *
  @verbatim
  ==============================================================================
    (1)bmi088_spi_dma_tx未明确定义
    (2)bmi088 spi_dma传输数据接口留出，但没有编写dma传输相关函数
  ==============================================================================
  @endverbatim
  *******************************RM Warrior 2023********************************
  */

#include "INS_Task.h"
#include "stm32f4xx.h"
#include "stdio.h"

#include "buzzer.h"
#include "led.h"
#include "spi.h"
#include "exit.h"
#include "adc.h"
#include "temperature.h"

#include "IST8310driver.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"

#include "AHRS.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t INSTaskStack;
#endif

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                    \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count);
static void imu_cali_solve(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

#if defined(BMI088_USE_TEMP_CONTROL)
static void imu_temp_control(fp32 temp);
#endif

#define IMU_temp_PWM(pwm)  bmi_pwm_set(pwm)                    //pwm给定
#define BMI088_DATA_READY_EXIT_INIT() exit_gyro_init()
#define IST8310_DATA_READY_EXIT_INIT() exit_line3_init()


//如果使用bmi088的数据准备外部中断，可以使用任务通知方法唤醒任务
#if defined(BMI088_USE_DATA_READY_EXIT) || defined(BMI088_USE_SPI_DMA)
static TaskHandle_t INSTask_Local_Handler;
#endif

/* Update_Flag
** BIT[0]      进入相应的data ready中断后置1
** BIT[1]      成功开启数据的SPI的DMA传输
** BIT[2]      已完成数据的SPI的DMA传输
** BIT[3-7]    保留
**
*/
uint8_t gyro_update_flag = 0;
uint8_t accel_update_flag = 0;
uint8_t accel_temp_update_flag = 0;
// uint8_t mag_update_flag = 0;
uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};//陀螺仪校准线性度
fp32 gyro_offset[3] = {0.0f, 0.0f, 0.0f};
fp32 gyro_cali_offset[3] = {0.0f, 0.0f, 0.0f};

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};//加速度计校准线性度
fp32 accel_offset[3] = {0.0f, 0.0f, 0.0f};

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};//磁力计校准线性度
fp32 mag_offset[3];


static uint8_t first_temperate = 0;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static PidTypeDef imu_temp_pid;


//加速度计低通滤波
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};


static const float timing_time = 0.001f;   //任务运行的时间 单位 s


static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};

int8_t temp_set = 0;

void INSTask(void *pvParameters)
{
    vTaskDelay(INS_TASK_INIT_TIME);

    while(BMI088_init())
    {
        ;
    }

#if defined(USE_IST8310)
    while(ist8310_init())
    {
        ;
    }
#endif

#if defined(USE_IST8310)
    IST8310_DATA_READY_EXIT_INIT();
#endif

#if defined(BMI088_USE_DATA_READY_EXIT) || defined(BMI088_USE_SPI_DMA)
    //获取当前任务的任务句柄，用于任务通知
    INSTask_Local_Handler = xTaskGetHandle(pcTaskGetName(NULL));
#endif


#if defined(BMI088_USE_DATA_READY_EXIT)
    //初始化bmi088的数据准备外部中断
    BMI088_DATA_READY_EXIT_INIT();
    exti_line1_init();
#else
    //如果不使用外部中断任务唤醒任务的方法，则使用传统的任务切换的方法
    TickType_t INS_LastWakeTime = xTaskGetTickCount();
#endif

//初始化SPI的DMA传输的方法
#if defined(BMI088_USE_SPI_DMA) && defined(BMI088_USE_SPI)
    imu_start_dma_flag = 1;
    /*
    **********************************************************
    */
#endif

//初始化陀螺仪加热
#if defined(BMI088_USE_TEMP_CONTROL)
    temp_pwm_init(BMI088_TEMP_PWM_MAX, 1);
    PID_Init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    temp_set = (int8_t)(get_temprate()) + 10;//根据mcu内部温度确定imu控制的温度
#endif

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

    imu_cali_solve(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

    AHRS_init(INS_quat, INS_accel, INS_mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];

    //设置spi的速度
    SPI1_SetSpeedAndDataSize(SPI_BaudRatePrescaler_8, SPI_DataSize_8b);


    while(1)
    {

#if defined(BMI088_USE_DATA_READY_EXIT)
        //等待外部中断唤醒任务
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
#else
        //任务延时切换任务
        vTaskDelayUntil(&INS_LastWakeTime, INS_DELTA_TICK);

#endif

//如果不使用SPI+DMA的方法，则使用普通SPI的方法
#ifndef BMI088_USE_SPI_DMA
        BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

#else
        /*
        **********************************************************
        */

#endif

        imu_cali_solve(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

        //加速度计低通滤波
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];


        AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

        //陀螺仪开机校准
        // {
        //     static uint16_t start_gyro_cali_time = 0;
        //     if(start_gyro_cali_time == 0)
        //     {
        //         gyro_offset[0] = gyro_cali_offset[0];
        //         gyro_offset[1] = gyro_cali_offset[1];
        //         gyro_offset[2] = gyro_cali_offset[2];
        //         start_gyro_cali_time++;
        //     }
        //     else if (start_gyro_cali_time < GYRO_OFFSET_START_TIME)
        //     {
        //         //IMUWarnBuzzerOn();
        //         if( first_temperate)
        //         {
        //             //当进入gyro_offset函数，如果无运动start_gyro_cali_time++，如果有运动 start_gyro_cali_time = 0
        //             // gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, &start_gyro_cali_time);
        //             gyro_offset_calc(gyro_offset, INS_gyro, &start_gyro_cali_time);
        //         }
        //     }
        //     else if (start_gyro_cali_time == GYRO_OFFSET_START_TIME)
        //     {

        //         //IMUWarnBuzzerOFF();
        //         start_gyro_cali_time++;
        //     }
        // }


#if defined(BMI088_USE_TEMP_CONTROL)     
        imu_temp_control(bmi088_real_data.temp);
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
        INSTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif

    }
}

/**
  * @brief          旋转陀螺仪、加速度计和磁力计，并计算零飘
  * @param[out]     gyro: 加上零飘和旋转
  * @param[out]     accel: 加上零飘和旋转
  * @param[out]     mag: 加上零飘和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_solve(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_Calc(&imu_temp_pid, temp, temp_set);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        if (temp > temp_set)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = BMI088_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(BMI088_TEMP_PWM_MAX - 1);
    }
}


/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @param[out]     offset_time_count: 自动加1
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - GYRO_OFFSET_KP * gyro[0];
        gyro_offset[1] = gyro_offset[1] - GYRO_OFFSET_KP * gyro[1];
        gyro_offset[2] = gyro_offset[2] - GYRO_OFFSET_KP * gyro[2];
        (*offset_time_count)++;
}


/**
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         none
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;
}


/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         none
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}


/**
  * @brief          获取四元数
  * @param[in]      none
  * @retval         INS_quat的指针
  */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}

/**
  * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
  * @param[in]      none
  * @retval         INS_angle的指针
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @param[in]      none
  * @retval         INS_accel的指针
  */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}

extern const fp32 *get_accel_filter_point(void)
{
    return accel_fliter_3;
}
/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
  * @param[in]      none
  * @retval         INS_mag的指针
  */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}

#if defined(USE_IST8310)
void EXTI3_IRQHandler(void)//ist8310 data ready
{
	if(EXTI_GetITStatus(EXTI_Line3) == SET)
	{
		ist8310_read_mag(ist8310_real_data.mag);
	}
	EXTI_ClearITPendingBit(EXTI_Line3);
}
#endif


#if defined(BMI088_USE_DATA_READY_EXIT)
//陀螺仪数据准备 中断服务函数
void EXTI4_IRQHandler(void)//accel
{
	if(EXTI_GetITStatus(EXTI_Line4) == SET)
	{
        // detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            // imu_cmd_spi_dma();
        }
        else
        {
            if( (accel_update_flag & (1 << IMU_DR_SHFITS))&&
                (accel_temp_update_flag & (1 << IMU_DR_SHFITS))&& 
                (gyro_update_flag & (1 << IMU_DR_SHFITS)) )
            {
                EXTI_GenerateSWInterrupt(EXTI_Line1);
            }
        }
	}
	EXTI_ClearITPendingBit(EXTI_Line4);
}

void EXTI9_5_IRQHandler(void)//gyro
{
	if(EXTI_GetITStatus(EXTI_Line5) == SET)
	{
        // detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            // imu_cmd_spi_dma();
        }
        else
        {
            if( (accel_update_flag & (1 << IMU_DR_SHFITS))&&
                (accel_temp_update_flag & (1 << IMU_DR_SHFITS))&& 
                (gyro_update_flag & (1 << IMU_DR_SHFITS)) )
            {
                EXTI_GenerateSWInterrupt(EXTI_Line1);
            }
        }
	}
	EXTI_ClearITPendingBit(EXTI_Line5);
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) == SET)
	{
        if(!imu_start_dma_flag)
        {
            accel_update_flag = accel_temp_update_flag = gyro_update_flag = 0;
        }
        // 唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR((INSTask_Local_Handler), &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        //注意中断优先级的设置
        //FreeRTOS为了满足某些应用对中断实时性要求高的需求
        //使得中断优先级高于某个值之后，就不能调用操作系统的内核函数来提高实时性
	}
	EXTI_ClearITPendingBit(EXTI_Line1);
}
#endif
