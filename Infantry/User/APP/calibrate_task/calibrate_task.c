/**
  ****************************RM Warrior 2023****************************
  * @file       校准设备，包括云台,陀螺仪,加速度计,磁力计,底盘.云台校准是主要计算零点
  * @brief      和最大最小相对角度.陀螺仪校准主要是计算零漂.加速度计和磁力计校准还没有实现
  *             因为加速度计还没有必要去校准,而磁力计还没有用.底盘校准是使M3508进入快速
  *             设置ID模式.
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-25-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis calibration
  *
  @verbatim
  ==============================================================================
  *             使用遥控器进行开始校准
  *             第一步:遥控器的两个开关都打到下
  *             第二步:两个摇杆打成\../,保存两秒。\.代表左摇杆向右下打.
  *             第三步:摇杆打成./\. 开始陀螺仪校准
  *                    或者摇杆打成'\/' 开始云台校准
  *                    或者摇杆打成/''\ 开始底盘校准
  *
  *             数据在flash中，包括校准数据和名字 name[3] 和 校准标志位 cali_flag
  *             例如head_cali有八个字节,但它需要12字节在flash,如果它从0x080A0000开始
  *             0x080A0000-0x080A0007: head_cali数据
  *             0x080A0008: 名字name[0]
  *             0x080A0009: 名字name[1]
  *             0x080A000A: 名字name[2]
  *             0x080A000B: 校准标志位 cali_flag,当校准标志位为0x55,意味着head_cali已经校准了
  *             添加新设备
  *             1.添加设备名在calibrate_task.h的cali_id_e, 像
  *             typedef enum
  *             {
  *                 ...
  *                 //add more...
  *                 CALI_XXX,
  *                 CALI_LIST_LENGHT,
  *             } cali_id_e;
  *             2. 添加数据结构在 calibrate_task.h, 必须4字节倍数，像
  *
  *             typedef struct
  *             {
  *                 uint16_t xxx;
  *                 uint16_t yyy;
  *                 fp32 zzz;
  *             } xxx_cali_t; //长度:8字节 8 bytes, 必须是 4, 8, 12, 16...
  *             3.在 "FLASH_WRITE_BUF_LENGHT",添加"sizeof(xxx_cali_t)", 和实现新函数
  *             bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd), 添加新名字在 "cali_name[CALI_LIST_LENGHT][3]"
  *             和申明变量 xxx_cali_t xxx_cail, 添加变量地址在cali_sensor_buf[CALI_LIST_LENGHT]
  *             在cali_sensor_size[CALI_LIST_LENGHT]添加数据长度, 最后在cali_hook_fun[CALI_LIST_LENGHT]添加函数
  *
  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

//包含校准设备的名字，校准标识符，校准数据flash大小，校准命令，对应的校准数据地址
#include "calibrate_task.h"

#include "adc.h"
#include "buzzer.h"
#include "flash.h"
#include "Remote_Control.h"
#include "INS_Task.h"
#include "string.h"
#include "gimbal_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

static void cali_data_read(void);                           //读取所有校准数据
static void cali_data_write(void);                          //写入当前校准变量的数据
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);   //表头校准数据函数
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);   //陀螺仪校准数据函数
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd); //云台校准数据函数

static const RC_ctrl_t *calibrate_RC; //遥控器结构体指针
static head_cali_t head_cali;         //表头校准数据
static gimbal_cali_t gimbal_cali;     //云台校准数据
static imu_cali_t gyro_cali;          //陀螺仪校准数据
static imu_cali_t accel_cali;         //加速度校准数据
static imu_cali_t mag_cali;           //磁力计校准数据


static cali_sensor_t cali_sensor[CALI_LIST_LENGHT]; //校准设备数组，

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"}; //校准设备的名字

//校准设备对应放入结构体变量地址
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] =
    {
        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
        (uint32_t *)&mag_cali};

//校准设备对应放入数据大小
static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

//校准设备对应的校准函数
void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};

//校准对应时间戳，利用freeRTOS的tick完成。
static uint32_t calibrate_systemTick;

//遥控器控制校准设备的校准
static void RC_cmd_to_calibrate(void);

void calibrate_task(void *pvParameters)
{
    static uint8_t i = 0;
    calibrate_RC = get_remote_ctrl_point_cali();

    while (1)
    {
        //遥控器操作校准步骤
        RC_cmd_to_calibrate();

        for (i = 0; i < CALI_LIST_LENGHT; i++)
        {
            //校准命令为1 表示需要校准
            if (cali_sensor[i].cali_cmd)
            {
                if (cali_sensor[i].cali_hook != NULL)
                {
                    //调用校准函数
                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {
                        //校准完成
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];

                        cali_sensor[i].cali_done = CALIED_FLAG;

                        cali_sensor[i].cali_cmd = 0;
                        //写入flash
                        cali_data_write();
                    }
                }
            }
        }
        vTaskDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
//返回bmi088需要控制到的温度
int8_t get_control_temperate(void)
{
    return head_cali.temperate;
}

//遥控器操作校准云台，陀螺仪
static void RC_cmd_to_calibrate(void)
{
    static const uint8_t BEGIN_FLAG   = 1;
    static const uint8_t GIMBAL_FLAG  = 2;
    static const uint8_t GYRO_FLAG    = 3;
    static const uint8_t CHASSIS_FLAG = 4;

    static uint8_t i;
    static uint32_t rc_cmd_systemTick = 0;
    static uint16_t buzzer_time = 0;
    static uint16_t rc_cmd_time = 0;
    static uint8_t rc_action_flag = 0;

    //如果设备正在校准 直接返回
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_cmd)
        {
            buzzer_time = 0;
            rc_cmd_time = 0;
            rc_action_flag = 0;
            return;
        }
    }

    if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //判断遥控器2s内八开始20s校准选择时间，rc_action_falg及rc_cmd_time在下方逻辑判断
        rc_cmd_systemTick = xTaskGetTickCount();
        rc_action_flag = BEGIN_FLAG;
        rc_cmd_time = 0;
    }
    else if (rc_action_flag == GIMBAL_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //判断遥控器在20s校准选择时间，上外八使能云台校准，并且保持2s,rc_action_falg及rc_cmd_time在下方逻辑判断
        rc_action_flag = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GIMBAL].cali_cmd = 1;
        //cali_buzzer_off();
    }
    else if (rc_action_flag == GYRO_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //判断遥控器在20s校准选择时间，下外八使能陀螺仪校准，并且保持2s，rc_action_falg及rc_cmd_time在下方逻辑判断
        rc_action_flag = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GYRO].cali_cmd = 1;
        //更新bmi088需要控制的温度
        head_cali.temperate = (int8_t)(cali_get_mcu_temperature()) + 10;
        if (head_cali.temperate > (int8_t)(GYRO_CONST_MAX_TEMP))
        {
            head_cali.temperate = (int8_t)(GYRO_CONST_MAX_TEMP);
        }
        //cali_buzzer_off();
    }
    else if (rc_action_flag == CHASSIS_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        rc_action_flag = 0;
        rc_cmd_time = 0;
        //发送CAN重设ID命令到3508
        CAN_CMD_CHASSIS_RESET_ID();
        CAN_CMD_CHASSIS_RESET_ID();
        CAN_CMD_CHASSIS_RESET_ID();
        cali_buzzer_off();
    }


    if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag == 0)
    {
        //两个摇杆打成下内八 \../, 保持2s
        rc_cmd_time++;
    }
    // else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
    // {
    //     //两个摇杆打成上外八'\/', 保持2s， 云台使能
    //     rc_cmd_time++;
    //     rc_action_flag = GIMBAL_FLAG;
    // }

    // else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
    // {
    //     //两个摇杆打成下外八 ./\., 保持2s， 陀螺仪使能
    //     rc_cmd_time++;
    //     rc_action_flag = GYRO_FLAG;
    // }
    else if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
    {
        //两个摇杆打成上内八 /''\, 保持2s
        rc_cmd_time++;
        rc_action_flag = CHASSIS_FLAG;
    }

    else
    {
        rc_cmd_time = 0;
    }

    calibrate_systemTick = xTaskGetTickCount();

    if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
    {
        //判断遥控器20s校准选择时间，无操作
        rc_action_flag = 0;
        return;
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {

        //判断遥控器10s后校准选择时间切换蜂鸣器高频声音
        rc_cali_buzzer_middle_on();
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {

        //遥控器10s前 开始蜂鸣器低频声音
        rc_cali_buzzer_start_on();
    }

    if (rc_action_flag != 0)
    {
        buzzer_time++;
    }

    //蜂鸣器断续发声
    if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_flag != 0)
    {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_flag != 0)
    {
        cali_buzzer_off();
    }
}

//初始化校准结构体数组，读取flash值，如果未校准，使能校准命令,同时初始化对应校准数据
void cali_param_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    cali_data_read();

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
            if (cali_sensor[i].cali_hook != NULL)
            {
                //如果校准完毕，则将校准值传递到对应的校准参数
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
}

void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //读取校准设备的前部数据
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        //将名字，校准标识符进行赋值
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];

        //flash位置偏移
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
        //将flash保存的数据，传递到设备对应的变量地址中
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        //偏移校准数据字节大小
        offset += cali_sensor[i].flash_len * 4;
        //如果设备未校准，使能校准
        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
        {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}

static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;
    const uint16_t len = (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3) / 4 + 5;
    uint8_t buf[4 * len];
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //复制设备前部参数，例如名字， 数据大小
        memcpy((void *)(buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;

        //复制设备校准数据
        memcpy((void *)(buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
        offset += cali_sensor[i].flash_len * 4;
    }

    //写入flash
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)buf, len);
}

/**
  * @brief          "head"设备校准，主要保存纬度，imu控制的温度，硬件版本号
  * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
{
    head_cali_t *local_cali_t = (head_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        return 1;
    }
    local_cali_t->self_id = SELF_ID;
    local_cali_t->temperate = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (local_cali_t->temperate > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
        local_cali_t->temperate = (int8_t)(GYRO_CONST_MAX_TEMP);
    }
    local_cali_t->firmware_version = FIRMWARE_VERSION;
    local_cali_t->latitude = Latitude_At_ShenZhen;
    return 1;
}

/**
  * @brief          陀螺仪设备校准，主要校准零漂
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count_time = 0;
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
            cali_buzzer_off();
            return 1;
        }
        else
        {
            gyro_cali_disable_control(); //掉线遥控器以防误操作
            imu_start_buzzer();
            return 0;
        }
    }
    return 0;
}

/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd)
{

    gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
                             local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
                             local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
                                 &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
                                 &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle))
        {
            cali_buzzer_off();
            return 1;
        }
        else
        {
            gimbal_start_buzzer();
            return 0;
        }
    }
    return 0;
}

//返回纬度信息
void getFlashLatitude(float *latitude)
{

    if (latitude == NULL)
    {
        return;
    }
    if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
    {
        *latitude = head_cali.latitude;
    }
    else
    {
        *latitude = Latitude_At_ShenZhen;
    }
}
